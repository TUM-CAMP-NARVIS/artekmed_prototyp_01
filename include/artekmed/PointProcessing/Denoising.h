#pragma once
#ifndef ARTEKMED_P1_DENOISING_H
#define ARTEKMED_P1_DENOISING_H

#include <Eigen/Core>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <vector>

namespace artekmed
{
	namespace pointcloud
	{

		/*
			Guided Denoising Approach by Han, Jin, Wang, Jiang 2017
			Extended for Weighted Points in the centroid-Calculation unfortunately doesn't provide normal
		*/
		template<typename NeighbourIterator, typename WeightIterator>
		Eigen::Vector3f guidedDenoisingWeigthedLocal(const Eigen::Vector3f &sourcePoint,
		                                             const NeighbourIterator &neighboursStart,
		                                             const NeighbourIterator &neighboursEnd,
		                                             const WeightIterator &weightsStart,
		                                             const WeightIterator &weightsEnd,
		                                             const float epsilon = 0.05f)
		{
			constexpr bool enableWeights = true;


			Eigen::Vector3f centroid = {0, 0, 0};
			float centroidSquaredNorm = 0;
			float weightSum = 0.f;
			auto point = neighboursStart;
			auto weight = weightsStart;
			size_t pointsCount = 0;

			while (point != neighboursEnd && weight != weightsEnd)
			{

				//With weighting
				if (enableWeights)
				{
					centroidSquaredNorm += (*point).squaredNorm() * (*weight);
					centroid += *point * (*weight);
				}
				else
				{
					centroidSquaredNorm += (*point).squaredNorm();
					centroid += *point;
				}

				weightSum += *weight;
				point++;
				weight++;
				pointsCount++;
			}
			if (enableWeights)
			{
				centroid /= weightSum;
				centroidSquaredNorm /= weightSum;
			}
			else
			{
				centroid /= pointsCount;
				centroidSquaredNorm /= pointsCount;
			}
			auto sumDiff = centroidSquaredNorm - centroid.squaredNorm();
			auto a = sumDiff / (sumDiff + epsilon);
			auto b = centroid - a * centroid;
			return a * sourcePoint + b;
		}





		//see http://www.cs.tau.ac.il/~dcor/online_papers/papers/points_set_vis01.pdf step 1 we assume is already done
		// but step 2 is analog
		void mlsNormalEstimationAndSmoothing(
			const std::vector<Eigen::Vector3f> &neighbourPoints,
			Eigen::Vector3f &point, // point we want to smooth
			Eigen::Vector3f &normal, //approximate input normal n
			const Eigen::Vector3f &centroid,
			float &outCurvatureFactor,
			const uint8_t degree = 2,
			const float h = 0.3f);

		void pcaNormalEstimation(
			const std::vector<Eigen::Vector3f> &neighbours,
			Eigen::Vector3f &outCentroid,
			Eigen::Vector3f &outNormal,
			float &sigma
		);
	}
}
#endif //ARTEKMED_P1_DENOISING_H