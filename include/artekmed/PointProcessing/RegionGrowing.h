
#ifndef ARTEKMED_P1_REGIONGROWING_H
#define ARTEKMED_P1_REGIONGROWING_H
//Base Class for Point Cloud to Point Cloud processing
#include <Core/Core.h>
#include <Eigen/Eigen>
#include "Common.h"

namespace artekmed
{
	namespace pointcloud
	{
		//Original Algorithm from the Paper, not very optimzed
		open3d::PointCloud regionGrowingResampleA(
			const std::vector<Eigen::Vector3d> &inputPointCloud,
			const std::vector<DepthImageSource> &depthImages,
			const int seed,
			const size_t numSamplesTarget
		);

		//Tries for a relaxed, not so correct version of the region growing algorithm, with hopefully better performance
		open3d::PointCloud regionGrowingResampleB(
			const std::vector<Eigen::Vector3d>& inputPointCloud,
			const std::vector<DepthImageSource>& depthImages,
			const int seed,
			const size_t numSamplesTarget
		);


		void queryNeighbours(std::vector <InputSampleCoordinate> &neighbourIndices,
												 const std::vector <DepthImageSource> &depthImages,
												 const Eigen::Vector3f & sourcePoint,
												 const float radius
		);

		//queries only roughly by (mis)using the image locality coupled with depth information
		void queryNNearestNeighbours(std::vector<Eigen::Vector3f> &neighbours,
		                             const std::vector<DepthImageSource> &depthImages,
		                             const Eigen::Vector3f &sourcePoint,
		                             const float maxNeighbourhoodRadius,
		                             const uint32_t n
		);

		void queryNeighboursSorted(
			std::vector <InputSampleCoordinate> &neighbours,
			const std::vector <DepthImageSource> &depthImages,
			const Eigen::Vector3f & sourcePoint,
			const float radius
		);
	}
}

#endif //ARTEKMED_P1_REGIONGROWING_H
