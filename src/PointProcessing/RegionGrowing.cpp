#include <random>
#include <unordered_map>
#include <Eigen/Eigenvalues>
#include "artekmed/PointProcessing/RegionGrowing.h"
#include "artekmed/PointCloudProcessing.h"
#include "artekmed/PointProcessing/Denoising.h"

namespace artekmed
{
	namespace pointcloud
	{

		void growRegion(
			Eigen::Vector3f &centroid,
			Eigen::Vector3f &v_2,
			std::unordered_map<InputSampleCoordinate, size_t, InputSampleCoordinate::hash> &regionMapping,
			const size_t nextRegionIndex,
			const std::vector<InputSampleCoordinate> &neighbourhood,
			const int maxRegionSize,
			const float sigma_max,
			const std::vector<DepthImageSource> &depthImages)
		{
			float sigma_n = 0;
			centroid = {0, 0, 0};
			//Add closest points one after the other, until we are maxSize or curvature is too high.
			for (int k = 1; k < std::min((size_t) maxRegionSize, neighbourhood.size()) && sigma_n < sigma_max; ++k)
			{
				//TODO Absolute preformance disaster.. should use Eigen::Map to prevent unnecessary allocation,
				//but that would require corret data alignment and this is too much of a hassle right now
				Eigen::MatrixXf observedMatrix(k, 3);
				centroid = {0, 0, 0};
				for (int j = 0; j < k; ++j)
				{
					auto point3d = neighbourhood[j].getPointIn3D(depthImages);
					centroid += point3d;
					observedMatrix.row(j) =
						Eigen::Map<Eigen::RowVector3f>(point3d.data(), 3);
				}
				centroid /= observedMatrix.rows();
				//Construct Covaraince Matrix:
				// https://stackoverflow.com/questions/15138634/eigen-is-there-an-inbuilt-way-to-calculate-sample-covariance
				const auto centered = observedMatrix.rowwise() - observedMatrix.colwise().mean();
				if (observedMatrix.rows() > 1)
				{
					Eigen::MatrixXf cov = (centered.adjoint() * centered) / float(observedMatrix.rows() - 1);
					auto solver = Eigen::EigenSolver<Eigen::MatrixXf>(cov);
					//Our PCA Eigenvalues
					auto lambda_0 = solver.eigenvalues()(0).real();
					auto lambda_1 = solver.eigenvalues()(1).real();
					auto lambda_2 = solver.eigenvalues()(2).real();
					//Our PCA Eigenvectors
					Eigen::Vector3f v_0 = solver.eigenvectors().col(0).real().cast<float>();
					Eigen::Vector3f v_1 = solver.eigenvectors().col(1).real().cast<float>();
					v_2 = solver.eigenvectors().col(2).real().cast<float>();
					//The Eigenvalues, eigenvectors are not sorted: sort them here from highest (v_0) to lowest(v_2)
					if (lambda_0 < lambda_1)
					{
						std::swap(lambda_0, lambda_1);
						std::swap(v_0, v_1);
					}
					if (lambda_1 < lambda_2)
					{
						std::swap(lambda_1, lambda_2);
						std::swap(v_1, v_2);
					}
					if (lambda_0 < lambda_1)
					{
						std::swap(lambda_0, lambda_1);
						std::swap(v_0, v_1);
					}
					sigma_n = lambda_2 / (lambda_0 + lambda_1 + lambda_2);
				}
				if (sigma_n < sigma_max)
				{
					auto findIt = regionMapping.find(neighbourhood[k]);
					if (findIt == regionMapping.end())
					{
						regionMapping[neighbourhood[k]] = nextRegionIndex;
					}
					//else -- point is already in another region
				}
			}
		}

		open3d::PointCloud regionGrowingResampleA(
			const std::vector<Eigen::Vector3d> &inputPointCloud,
			const std::vector<DepthImageSource> &depthImages,
			const int seed,
			const size_t numSamplesTarget)
		{

			constexpr float neighbourhoodRadius = 0.02f;
			constexpr int maxRegionSize = 1500;
			constexpr float sigma_max = 0.15f;
			constexpr int SORminValue =20;

			open3d::PointCloud output;

			//We have to keep track, which input samples have already been added to a cluster
			std::unordered_map<InputSampleCoordinate, size_t, InputSampleCoordinate::hash> regionMapping;

			//Initial raw sampling: take one tenth of our total number of samples randomly
			const size_t initialNumSamples = numSamplesTarget /3;
			const size_t numInputPoints = inputPointCloud.size();
			std::default_random_engine generator(seed);
			std::uniform_int_distribution<size_t> distribution(0, numInputPoints - 1);

			for (size_t i = 0; i < initialNumSamples; ++i)
			{
				const auto randomIndex = distribution(generator);

				Eigen::Vector3f newPoint = inputPointCloud[randomIndex].cast<float>();
				std::vector<InputSampleCoordinate> neighbourhood = {};
				queryNeighboursSorted(neighbourhood, depthImages, newPoint, neighbourhoodRadius);
				//std::cout << neighbourhood.size() << '\n';
				Eigen::Vector3f centroid;
				Eigen::Vector3f v_2;
				if (neighbourhood.size()>=SORminValue)
				{
					growRegion(centroid, v_2, regionMapping, i, neighbourhood, maxRegionSize, sigma_max, depthImages);
					output.points_.emplace_back(centroid.cast<double>());
					const auto greyscale = neighbourhood.size()/255.0;
					output.colors_.emplace_back(greyscale,greyscale,greyscale);
					output.normals_.emplace_back(v_2.normalized().cast<double>());
				} else // Did not find any
				{
					i--;
					continue;
				}
			}

			//Refinining: take random regions and add a new region besides them..
			for (auto remainingSamples = numSamplesTarget - initialNumSamples; remainingSamples > 0; --remainingSamples)
			{
				distribution = std::uniform_int_distribution<size_t>(0, output.points_.size() - 1);
				Eigen::Vector3f regionToGrow = output.points_[distribution(generator)].cast<float>();
				//Find the next point which is not part of the region
				std::vector<InputSampleCoordinate> neighbourhood = {};
				//TODO Good Performance boost to get here by adding early termination
				queryNeighbours(neighbourhood, depthImages, regionToGrow, neighbourhoodRadius * 2);
				int nextRegionOrigin = 0;
				if(neighbourhood.empty()){
					remainingSamples++;
					continue;
				}
				while (regionMapping.find(neighbourhood[nextRegionOrigin]) != regionMapping.end() &&
				       nextRegionOrigin < neighbourhood.size())
				{
					++nextRegionOrigin;
				}
				if (nextRegionOrigin == neighbourhood.size())
				{
					//not good. we tried to find a free point in the neighbourhood but didt find any
					remainingSamples++;
					continue;
				}
				Eigen::Vector3f newPoint = neighbourhood[nextRegionOrigin].getPointIn3D(depthImages);
				std::vector<InputSampleCoordinate> newNeighbourhood = {};
				queryNeighbours(newNeighbourhood, depthImages, newPoint, neighbourhoodRadius);
				if(newNeighbourhood.size() >= SORminValue)
				{
					Eigen::Vector3f centroid;
					Eigen::Vector3f v_2;
					growRegion(centroid, v_2, regionMapping, output.points_.size(), newNeighbourhood, maxRegionSize, sigma_max,
					           depthImages);
					output.points_.emplace_back(centroid.cast<double>());
					const auto greyscale = newNeighbourhood.size() / 255.0;
					output.colors_.emplace_back(greyscale, greyscale, greyscale);
					output.normals_.emplace_back(v_2.normalized().cast<double>());
				}
				else{
					remainingSamples++;
					continue;
				}
				std::cout << remainingSamples<<'\n';
			}

			std::cout << "Finished\n";

			return output;
		}

		void subdivideRegion(
			const std::vector<Eigen::Vector3f> & neighbours, 
			open3d::PointCloud &output, 
			const float sigma_max,
			const uint32_t minRegionSize)
		{
			Eigen::Vector3f centroid;
			Eigen::Vector3f normal;
			const auto pca = pcaEigenValues(neighbours, centroid, normal);
			if (getSigma(pca) > sigma_max && minRegionSize < neighbours.size())
			{
				//Subdivide along centroid and v0
				std::vector<Eigen::Vector3f> planeFront;
				std::vector<Eigen::Vector3f> planeBack;
				for (const auto& n : neighbours)
				{
					if (normal.dot(n - centroid) >= 0)
					{
						planeFront.push_back(n);
					}
					else
					{
						planeBack.push_back(n);
					}
				}
				subdivideRegion(planeFront, output, sigma_max,minRegionSize);
				subdivideRegion(planeBack, output, sigma_max,minRegionSize);
			}
			else {
				output.points_.emplace_back(centroid.cast<double>());
				output.normals_.emplace_back(normal.cast<double>());
			}
		}

		open3d::PointCloud regionGrowingResampleB(
			const std::vector<Eigen::Vector3d>& inputPointCloud,
			const std::vector<DepthImageSource>& depthImages,
			const int seed,
			const size_t numSamplesTarget)
		{
			constexpr float neighbourhoodRadius = 0.02f;
			constexpr int maxRegionSize = 1500;
			constexpr int minRegionSize = 100;
			constexpr float sigma_max = 0.15f;
			constexpr int SORminValue = 20;

			std::default_random_engine generator(seed);
			std::uniform_int_distribution<size_t> distribution(0, inputPointCloud.size() - 1);

			open3d::PointCloud output;

			while (output.points_.size() < numSamplesTarget)
			{
				const auto randomIndex = distribution(generator);
				const auto basePoint = inputPointCloud[randomIndex].cast<float>();
				std::vector<Eigen::Vector3f> neighbours;
				queryNNearestNeighbours(neighbours, depthImages, basePoint, maxRegionSize);
				subdivideRegion(neighbours, output,sigma_max, minRegionSize);
			}
			return output;
		}


		void queryNeighbours(std::vector<InputSampleCoordinate> &neighbourIndices,
		                     const std::vector<DepthImageSource> &depthImages,
		                     const Eigen::Vector3f &sourcePoint,
		                     const float radius
		)
		{
			uint8_t imageIndex = 0;
			for (const auto &i : depthImages)
			{
				Eigen::Vector2i uvBase = {};
				if (project_point_to_pixel(uvBase, sourcePoint, i, radius))
				{
					const float depth = i.depthImage->at<float>((int) uvBase.y(), (int) uvBase.x()) * i.depthScaleFactor;
					const auto radius2d = radius * std::max(i.intrinsics(0, 0), i.intrinsics(1, 1)) / depth;
					const auto radius2dSquared = radius2d * radius2d;
					const auto radiusSquared = radius * radius;
					//Use a circle rasterization to find the pixels in radius2d around uvBase
					for (int x = std::max((int) (uvBase.x() - radius2d), 0);
					     x < std::min((int) (uvBase.x() + radius2d) + 1, i.depthImage->cols); ++x)
					{
						const auto xlen = std::abs(x - uvBase.x());
						if (xlen < radius2d)
						{
							const auto h = std::sqrt(radius2dSquared - xlen * xlen);
							for (int y = std::max((int) (uvBase.y() - h),0);
							     y < std::min((int) (uvBase.y() + h), i.depthImage->rows - 1) + 1; ++y)
							{
								InputSampleCoordinate potentialNeighbour = {x, y, imageIndex};
								const auto pointCandidate = potentialNeighbour.getPointIn3D(depthImages);
								if ((pointCandidate - sourcePoint).squaredNorm() < radiusSquared)
								{
									neighbourIndices.push_back(potentialNeighbour);
								}
							}
						}
					}
				}
				imageIndex++;
			}
		}
		struct DepthMapSampleUV
		{
			Eigen::Vector2i uv;
			float depth;
			const DepthImageSource* img;
			int currentRadius;

		};

		void addSamplesRing(std::vector<Eigen::Vector3f>& neighbours, DepthMapSampleUV & origin)
		{
			if(origin.currentRadius==0)
			{
				Eigen::Vector3f newPoint;
				const float dval = origin.img->depthImage->at<float>(origin.uv.y(),origin.uv.x());
				deproject_pixel_to_point(newPoint,
					origin.img->intrinsics,
					origin.uv.cast<float>(),
					dval*origin.img->depthScaleFactor);
				neighbours.emplace_back(newPoint);
			}
			else
			{
				const auto squaredRad = origin.currentRadius * origin.currentRadius;
				const auto squaredRadMinusOne = (origin.currentRadius - 1) * (origin.currentRadius - 1);
				//Rasterize a ring.. we floor the uv-coordantes (i.e. radius 1 will result in the 4 direct neighbours,
				// diagnoals are in in radius 2.
				for (int u = -origin.currentRadius; u <= origin.currentRadius; u++)
				{
					int from;
					if(squaredRadMinusOne >= u * u)
					{
						from = std::floor(std::sqrt(squaredRadMinusOne - u * u)) + 1;
					}
					else
					{
						from = 0;
					}
					const auto to = std::ceil(std::sqrt(squaredRad - u * u));
					for (int v = from; v <= to; ++v)
					{
						Eigen::Vector3f newPointA;
						deproject_pixel_to_point(
							newPointA,
							origin.img->intrinsics,
							{origin.uv.x() + u, origin.uv.y() + v},
							origin.img->depthImage->at<float>(v, u) * origin.img->depthScaleFactor);
						neighbours.emplace_back(std::move(newPointA));
						if(v !=0)
						{
							Eigen::Vector3f newPointB;
							deproject_pixel_to_point(
								newPointB,
								origin.img->intrinsics,
								{origin.uv.x() + u, origin.uv.y() - v},
								origin.img->depthImage->at<float>(v, u) * origin.img->depthScaleFactor);
							neighbours.emplace_back(std::move(newPointB));
						}
					}
				}
			}
		}

		void queryNNearestNeighbours(std::vector<Eigen::Vector3f> &neighbours,
		                             const std::vector<DepthImageSource> &depthImages,
		                             const Eigen::Vector3f &sourcePoint,
		                             const uint32_t n)
		{
			neighbours.clear();
			neighbours.reserve(n);

			std::vector<DepthMapSampleUV> depthValueAt;
			for (auto& i : depthImages)
			{
				Eigen::Vector2i uv;
				if (project_point_to_pixel(uv, sourcePoint, i))
				{
					depthValueAt.emplace_back(DepthMapSampleUV{
						uv,i.depthImage->at<float>(uv.y(),uv.x())*i.depthScaleFactor ,&i,0});
				}
			}
			std::sort(depthValueAt.begin(), depthValueAt.end(), [](const DepthMapSampleUV& a, const DepthMapSampleUV& b) {
				return a.depth < b.depth;
			});
			int currentIndex = 0;
			if(depthValueAt.size()==0)
			{
				return;
			}
			addSamplesRing(neighbours,depthValueAt[currentIndex]);
			depthValueAt[currentIndex].currentRadius++;
			while (neighbours.size() < n)
			{
				//Approximate the 3D radius of an image circle by taking the depth value of the midpoint and intrinsics
				const float nextRadiusInCurrentImage = 
					(depthValueAt[currentIndex].currentRadius +1.f) *
					(std::max(depthValueAt[currentIndex].img->intrinsics(0, 0), depthValueAt[currentIndex].img->intrinsics(1, 1)))
					/ depthValueAt[currentIndex].depth * depthValueAt[currentIndex].img->depthScaleFactor;
				const auto nextImageIdx = (currentIndex+1) % depthValueAt.size();
				const float nextRadiusInNextImage = 
					depthValueAt[nextImageIdx].currentRadius *
					(std::max(depthValueAt[nextImageIdx].img->intrinsics(0, 0), depthValueAt[nextImageIdx].img->intrinsics(1, 1)))
					/ depthValueAt[nextImageIdx].depth * depthValueAt[nextImageIdx].img->depthScaleFactor;

				const auto prevImageIdx = std::max(0, --currentIndex);
				const float nextRadiusInPrevImage = 
					depthValueAt[prevImageIdx].currentRadius *
					(std::max(depthValueAt[prevImageIdx].img->intrinsics(0, 0), depthValueAt[prevImageIdx].img->intrinsics(1, 1)))
					/ depthValueAt[prevImageIdx].depth * depthValueAt[prevImageIdx].img->depthScaleFactor;

				if (nextRadiusInCurrentImage < nextRadiusInNextImage) 
				{
					//check if going back would be smarter
					if (nextRadiusInPrevImage < nextRadiusInCurrentImage && currentIndex != 0)
					{
						currentIndex--;
						continue;
					}
					addSamplesRing(neighbours, depthValueAt[currentIndex]);
					depthValueAt[currentIndex].currentRadius++;
				}
				else
				{
					addSamplesRing(neighbours,depthValueAt[nextImageIdx]);
					depthValueAt[nextImageIdx].currentRadius++;
					currentIndex = nextImageIdx;
				}
			}
			//The target number of samples will be bigger with the current logic, strip them away.
			neighbours.resize(n);
		}

		//MW: Instead of returning any points in the neighbourhood, we get them back sorted by smallest distance to the
		// original point definately dont want to do this in a performant implementation but its nice now.
		void queryNeighboursSorted(
			std::vector<InputSampleCoordinate> &neighbours,
			const std::vector<DepthImageSource> &depthImages,
			const Eigen::Vector3f &sourcePoint,
			const float radius
		)
		{
			queryNeighbours(neighbours, depthImages, sourcePoint, radius);
			std::sort(neighbours.begin(), neighbours.end(),
			          [&sourcePoint, &depthImages]
				          (const InputSampleCoordinate &a, const InputSampleCoordinate &b) {
				          const auto apoint = a.getPointIn3D(depthImages);
				          const auto bpoint = b.getPointIn3D(depthImages);

				          return (sourcePoint - apoint).squaredNorm() < (sourcePoint - bpoint).squaredNorm();
			          });
		}
	}
}