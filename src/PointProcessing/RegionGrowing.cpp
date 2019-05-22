#include <random>
#include <unordered_map>
#include <Eigen/Eigenvalues>
#include "artekmed/PointProcessing/RegionGrowing.h"
#include "artekmed/PointCloudProcessing.h"

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

		open3d::PointCloud regionGrowingResample(
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