#include <random>
#include <unordered_map>
#include <Eigen/Eigenvalues>
#include "artekmed/PointProcessing/RegionGrowing.h"

namespace artekmed {
	namespace pointcloud {

		Eigen::Vector3f InputSampleCoordinate::getPointIn3D(const std::vector<DepthImageSource> & inputImages)const{
			Eigen::Vector3f result;
			::deproject_pixel_to_point(
				result,
				inputImages[imageIndex].intrinsics,
				{x,y},
				inputImages[imageIndex].depthImage->at<float>(x,y)
				);
			return result;
		}

		open3d::PointCloud regionGrowingResample(
			const open3d::PointCloud &inputPointCloud,
			const std::vector<DepthImageSource> &depthImages,
			const int seed,
			const size_t numSamplesTarget) {

			constexpr float neighbourhoodRadius=0.3f;
			constexpr int maxRegionSize= 50;
			constexpr float sigma_max = 0.2f;

			open3d::PointCloud output;

			//We have to keep track, which input samples have already been added to a cluster
			std::unordered_map<InputSampleCoordinate,size_t,InputSampleCoordinate::hash> regionMapping;

			//Initial raw sampling: take one tenth of our total number of samples randomly
			const size_t initialNumSamples = numSamplesTarget /10;
			const size_t numInputPoints = inputPointCloud.points_.size();
			std::default_random_engine generator(seed);
			std::uniform_int_distribution<size_t>distribution(0,numInputPoints-1);

			for(size_t i = 0; i < initialNumSamples;++i)
			{
				const auto randomIndex = distribution(generator);
				Eigen::Vector3f newPoint = inputPointCloud.points_[randomIndex].cast<float>();
				std::vector<InputSampleCoordinate> neighbourhood = {};
				queryNeighboursSorted(neighbourhood,depthImages,newPoint, neighbourhoodRadius);
				float sigma_n=0;
				Eigen::Vector3f centroid={0,0,0};
				for(int k = 1; k < maxRegionSize&& sigma_n < sigma_max; ++k)
				{
					//TODO Absolute preformance disaster.. should use Eigen::Map to prevent unnecessary allocation,
					//but that would require corret data alignment and this is too much of a hassle right now
					Eigen::MatrixXf observedMatrix(k,3);
					centroid={0,0,0};
					for(int j = 0; j<= k;++j)
					{
						auto point3d = neighbourhood[j].getPointIn3D(depthImages);
						centroid += point3d;
						observedMatrix.row(j) =
							Eigen::Map<Eigen::RowVector3f>(point3d.data(),3);
					}
					centroid /= observedMatrix.rows();
					//Construct Covaraince Matrix:
					// https://stackoverflow.com/questions/15138634/eigen-is-there-an-inbuilt-way-to-calculate-sample-covariance
					const auto centered = observedMatrix.rowwise()-observedMatrix.colwise().mean();
					if(observedMatrix.rows()>1) {
						Eigen::MatrixXf cov = (centered.adjoint() * centered) / float(observedMatrix.rows() - 1);
						auto solver = Eigen::EigenSolver<Eigen::MatrixXf>(cov);
						//Our PCA Eigenvalues
						auto lambda_0 = solver.eigenvalues()(0).real();
						auto lambda_1 = solver.eigenvalues()(1).real();
						auto lambda_2 = solver.eigenvalues()(2).real();
						//Our PCA Eigenvectors
						auto v_0 = solver.eigenvectors()(0);
						auto v_1 = solver.eigenvectors()(1);
						auto v_2 = solver.eigenvectors()(2);
						//The Eigenvalues, eigenvectors are not sorted from eigen: sort them here from highest (v_0) to lowest(v_1)
						if (lambda_0 < lambda_1) {
							std::swap(lambda_0, lambda_1);
							std::swap(v_0, v_1);
						}
						if (lambda_1 < lambda_2) {
							std::swap(lambda_1, lambda_2);
							std::swap(v_1, v_2);
						}
						if (lambda_0 < lambda_1) {
							std::swap(lambda_0, lambda_1);
							std::swap(v_0, v_1);
						}
						sigma_n = lambda_2/(lambda_0+lambda_1+lambda_2);
					}
					if(sigma_n < sigma_max){
						regionMapping.
					}
				}
			}
			auto remainingSamples = numSamplesTarget-initialNumSamples;

			return output;
		}

		bool project_point_to_pixel(
			Eigen::Vector2i &pixel,
			const Eigen::Vector3f &point,
			const DepthImageSource &depth_image,
			const float radius)
			{
			float x = point(0) / point(2);
			float y = point(1) / point(2);

			x = x * depth_image.intrinsics(0, 0) + depth_image.intrinsics(0, 2);
			y = y * depth_image.intrinsics(1, 1) + depth_image.intrinsics(1, 2);
			if (x < 0 || x > depth_image.depthImage->cols || y < 0 || y > depth_image.depthImage->rows) {
				//We are outside this image Space
				return false;
			}
			float depth = depth_image.depthImage->at<float>((int)x, (int)y);
			const auto zEpslion = radius/depth;
			if (std::abs(depth - point(2)) > zEpslion) {
				//Z coordinate does not fit into the depth value: point was occluded
				return false;
			}
			pixel(0) = (int)x;
			pixel(1) = (int)y;
			return true;
		}

		void queryNeighbours(std::vector <InputSampleCoordinate> &neighbourIndices,
												 const std::vector <DepthImageSource> &depthImages,
												 const Eigen::Vector3f & sourcePoint,
												 const float radius
		)
		{
			uint8_t imageIndex = 0;
			for(const auto & i : depthImages){
				Eigen::Vector2i uvBase = {};
				if(project_point_to_pixel(uvBase, sourcePoint,i,radius))
				{
					const auto radius2d = std::min(radius/i.depthImage->at<float>(uvBase.x(), uvBase.y()),1.f);
					const auto radius2dSquared = radius2d*radius2d;
					const auto radiusSquared = radius*radius;
					//Use a circle rasterization to find the pixels in radius2d around uvBase
					for(int x = std::max((int)(uvBase.x()-radius2d),0); x< std::min((int)(uvBase.x()+radius2d)+1, i.depthImage->cols);++x){
						const auto h = std::sqrt(radius2dSquared-(x-uvBase.x())*(x-uvBase.x()));
						for(int y = (int)(uvBase.y()-h); y < (int)(uvBase.y()+h)+1;++y)
						{
							InputSampleCoordinate potentialNeighbour = {x,y,imageIndex};
							if((potentialNeighbour.getPointIn3D(depthImages)-sourcePoint).squaredNorm()< radiusSquared){
								neighbourIndices.push_back(potentialNeighbour);
							}
						}
					}
				}
				imageIndex++;
			}
		}
		//Instead of returning any points in the neighbourhood, we get them back sorted by smallest distance to the
		// original point
		void queryNeighboursSorted(
			std::vector <InputSampleCoordinate> &neighbours,
			 const std::vector <DepthImageSource> &depthImages,
			 const Eigen::Vector3f & sourcePoint,
			 const float radius
		)
		{
			queryNeighbours(neighbours,depthImages,sourcePoint,radius);
			std::sort(neighbours.begin(), neighbours.end(),
				[&sourcePoint,&depthImages]
				(const InputSampleCoordinate & a, const InputSampleCoordinate & b){
				const auto apoint = a.getPointIn3D(depthImages);
				const auto bpoint = b.getPointIn3D(depthImages);

				return (sourcePoint-apoint).squaredNorm() <(sourcePoint-bpoint).squaredNorm();
			});
		}
	}
}