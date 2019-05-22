#include <random>
#include <algorithm>
#include <unordered_set>
#include <artekmed/PointProcessing/ClopAdapter.h>

#define CLOP_DEBUG 0

#include "artekmed/PointProcessing/ClopAdapter.h"
#include "artekmed/PointProcessing/clopdemo/mixture.hpp"
#include "artekmed/PointProcessing/clopdemo/clop.hpp"

namespace artekmed
{
	namespace pointcloud
	{
		void clopResampling(
			open3d::PointCloud &output,
			const open3d::PointCloud &inputCloud,
			const ClopResamplingParameters &clopParams
		)
		{
			//Probably unnecessary copy
			cp::PointSet originalPoints = {};
			for (const auto &p : inputCloud.points_)
			{
				originalPoints.emplace_back(p.x(), p.y(), p.z());
			}


			cp::PointSet particles = {};
			std::default_random_engine generator(clopParams.seed);
			std::uniform_int_distribution<size_t> distribution(0, inputCloud.points_.size() - 1);
			std::unordered_set<size_t> usedIndices = {};
			for (int i = 0; i < std::min(clopParams.nTargetSamples, inputCloud.points_.size() / 2); ++i)
			{
				size_t idx = 0;
				do
				{
					idx = distribution(generator);
				} while (usedIndices.find(idx) != usedIndices.end());
				usedIndices.insert(idx);
				particles.emplace_back(
					inputCloud.points_[idx].x(),
					inputCloud.points_[idx].y(),
					inputCloud.points_[idx].z()
				);
			}

			cp::Mixture::Params mParams;
			mParams.globalInitRadius = clopParams.mixture.globalInitRadius;
			mParams.useGlobalInitRadius = clopParams.mixture.useGlobalInitRadius;
			mParams.useWeightedPotentials = clopParams.mixture.useWeightedPotentials;
			mParams.alpha = clopParams.mixture.alpha;
			mParams.nLevels = clopParams.mixture.nLevels;

			cp::clop::Params parameters;
			parameters.nIterations = clopParams.nIterations;
			parameters.kernelRadius = clopParams.kernelRadius;
			parameters.doubleInitRadius = clopParams.doubleInitRadius;
			parameters.interleaveRepulsion = clopParams.interleaveRepulsion;
			parameters.repulsionRadiusFac = clopParams.repulsionRadiusFac;
			parameters.useSoftEta = clopParams.useSoftEta;
			parameters.mu = clopParams.mu;
			parameters.useDiscreteLOP = clopParams.useDiscreteLOP;

			cp::Mixture m = {&originalPoints, mParams};
			cp::PointSet *projectedPoints = cp::clop::project(&particles, &m, parameters);

			output.points_.clear();
			for (const auto &point: *projectedPoints)
			{
				output.points_.emplace_back(point.x, point.y, point.z);
			}

			delete projectedPoints;
		}
	}
}