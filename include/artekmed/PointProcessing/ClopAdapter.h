#ifndef ARTEKMED_P1_CLOPADAPTER_H
#define ARTEKMED_P1_CLOPADAPTER_H
/*
 * CLOP Algorithm from Preiner et. al
 */
#include <Core/Core.h>
#include "Common.h"
namespace artekmed
{
	namespace pointcloud
	{
		struct ClopResamplingParameters
		{
			struct{
				float globalInitRadius;
				bool useGlobalInitRadius;
				bool useWeightedPotentials;
				float alpha;
				uint8_t nLevels;
			} mixture;
			uint8_t nIterations;
			float kernelRadius;
			bool doubleInitRadius;
			bool interleaveRepulsion;
			float repulsionRadiusFac;
			bool useSoftEta;
			float mu;
			bool useDiscreteLOP;


			size_t nTargetSamples;
			uint64_t seed;
		};
		void clopResampling(
			open3d::PointCloud & output,
			const open3d::PointCloud & inputCloud,
			const ClopResamplingParameters & clopParams
		);
	}
}

#endif //ARTEKMED_P1_CLOPADAPTER_H
