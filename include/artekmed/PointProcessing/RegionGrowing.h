
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
		open3d::PointCloud regionGrowingResample(
			const std::vector<Eigen::Vector3d> &inputPointCloud,
			const std::vector<DepthImageSource> &depthImages,
			const int seed,
			const size_t numSamplesTarget
		);


		void queryNeighbours(std::vector <InputSampleCoordinate> &neighbourIndices,
												 const std::vector <DepthImageSource> &depthImages,
												 const Eigen::Vector3f & sourcePoint,
												 const float radius
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
