//
// Created by narvis on 14.05.19.
//

#ifndef ARTEKMED_P1_VOXELRESAMPLING_H
#define ARTEKMED_P1_VOXELRESAMPLING_H

#include <Core/Geometry/PointCloud.h>
#include "Common.h"

namespace artekmed
{
	namespace pointcloud
	{
		void voxelResampling(
			open3d::PointCloud & output,
			const std::vector<DepthImageSource> & imageSource,
			const open3d::PointCloud & inputCloud,
			float minHalfSize = 0.01f,
			float maxHalfSize = 0.1f
			);
	}
}

#endif //ARTEKMED_P1_VOXELRESAMPLING_H
