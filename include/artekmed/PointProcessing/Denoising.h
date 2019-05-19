#pragma once

namespace artekmed
{
	namespace pointcloud
	{
		
		/*
			Guided Denoising Approach by Han, Jin, Wang, Jiang 2017
			Extended for Weighted Points in the centroid-Calculation
		*/
		template<typename NeighbourIterator, typename WeightIterator>
		Eigen::Vector3f guidedDenoisingWeigthedLocal(const Eigen::Vector3f& sourcePoint,
			const NeighbourIterator& neighboursStart,
			const NeighbourIterator& neighboursEnd,
			const WeightIterator& weightsStart,
			const WeightIterator& weightsEnd,
			const float epsilon = 0.05f)
		{
			constexpr bool enableWeights = true; 


			Eigen::Vector3f centroid = { 0,0,0 };
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
			auto sumDiff = centroidSquaredNorm - centroid.squaredNorm()
			auto a = sumDiff / (sumDiff + epsilon);
			auto b = centroid - a * centroid;
			return a * sourcePoint + b;
		}

		template<class VectorItType>
		void mlsNormalEstimation(
			std::vector<Eigen::Vector3f>& inputPoints,
			Eigen::Vector3f& test,
			const size_t mainPointIndex,
			const uint8_t degree = 1,
			const Eigen::Vector3f& initialEstimate = { 1,0,0 },
			const Eigen::Vector3f& centroid = { -1,0,0 })
		{

		}
	}
}