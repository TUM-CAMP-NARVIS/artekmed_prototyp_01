#include <cmath>
#include <mutex>
#include "artekmed/PointProcessing/VoxelResampling.h"

#include "artekmed/PointCloudProcessing.h"
#include "artekmed/Utils/TemperatureColors.h"
#include "artekmed/PointProcessing/Denoising.h"

namespace artekmed
{
	namespace pointcloud
	{
#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-flp30-c"

		float maxAbsValue(const Eigen::Vector3f &v)
		{
			return std::max(std::abs(v(0)), std::max(std::abs(v(1)), std::abs(v(2))));
		}

		//Neighbour query, that considers a cube around a base point
		void
		queryNeighboursCube(std::vector<Eigen::Vector3f> &outputPoints,
		                    const std::vector<DepthImageSource> &imageSource,
		                    const Eigen::Vector3f &sourcePoint,
		                    const float &halfSize
		)
		{
			Eigen::Vector2i uvCoords = {};
			for (const auto &i : imageSource)
			{
				if (project_point_to_pixel(uvCoords, sourcePoint, i, halfSize))
				{
					const float depth = i.depthImage->at<float>((int) uvCoords.y(), (int) uvCoords.x()) * i.depthScaleFactor;
					if (isnan(depth))
					{
						return;
					}
					const auto radius2d = halfSize * std::max(i.intrinsics(0, 0), i.intrinsics(1, 1)) / depth;
					const auto radius2dSquared = radius2d * radius2d;
					//Use a circle rasterization to find the pixels in radius2d around uvCoords
					for (int x = std::max((int) (uvCoords.x() - radius2d), 0);
					     x < std::min((int) (uvCoords.x() + radius2d) + 1, i.depthImage->cols); ++x)
					{
						const auto xlen = std::abs(x - uvCoords.x());
						if (xlen < radius2d)
						{
							const auto h = std::sqrt(radius2dSquared - xlen * xlen);
							for (int y = std::max((int) (uvCoords.y() - h), 0);
							     y < std::min((int) (uvCoords.y() + h), i.depthImage->rows - 1) + 1; ++y)
							{
								Eigen::Vector3f candidate3D;
								const float candidateDepth = i.depthImage->at<float>(y, x) * i.depthScaleFactor;
								if (isnan(candidateDepth))
								{
									continue;
								}
								::deproject_pixel_to_point(candidate3D, i.intrinsics, {x, y}, candidateDepth);
								if (maxAbsValue(candidate3D - sourcePoint) <= halfSize)
								{
									outputPoints.push_back(candidate3D);
								}
							}
						}
					}
				}
			}
		}

		constexpr float sigma_max = 15;//0.002f;
		constexpr uint32_t SORminValue = 4;

		Eigen::Vector3f octreeNextNodePos(const Eigen::Vector3f &base, const float newHalfSize, const uint8_t index)
		{
			return base + Eigen::Vector3f{
				CHECK_BIT(index, 0) ? -newHalfSize : newHalfSize,
				CHECK_BIT(index, 1) ? -newHalfSize : newHalfSize,
				CHECK_BIT(index, 2) ? -newHalfSize : newHalfSize
			};
		}

		//Recursive, cause easier to implement
		void doBuildVoxelResampling(open3d::PointCloud &output,
		                            const Eigen::Vector3f &base,
		                            const float halfSize,
		                            const float halfSizeMin,
		                            const float sigma_max,
		                            const std::vector<DepthImageSource> &sourceImages)
		{
			std::vector<Eigen::Vector3f> neighbours;
			queryNeighboursCube(neighbours, sourceImages, base, halfSize);
			if (neighbours.size() > SORminValue)
			{
				Eigen::Vector3f centroid;
				Eigen::Vector3f normal;
				float sigma_n;
				pcaNormalEstimation(neighbours,centroid,normal,sigma_n);
				Eigen::Vector3f newPoint = base;
				float error=sigma_n;
				mlsNormalEstimationAndSmoothing(neighbours,newPoint,normal,centroid,error,3,halfSize);
				//std::cout << "Curvature: "<<error<<'\n';
				if (error > sigma_max && halfSizeMin < halfSize / 2)
				{
					//not enough precision: Subdivide this Cube further
					for (uint8_t i = 0; i < 8; ++i)
					{
						const auto nextPos = octreeNextNodePos(base, halfSize / 2, i);
						doBuildVoxelResampling(output, nextPos, halfSize / 2, halfSizeMin, sigma_max, sourceImages);
					}
				}
				else
				{
					//FIXME estimate unitl we have the correct Extrinsics
					alignNormal(normal,centroid,{0,-2,0});
					output.points_.emplace_back(centroid.cast<double>());
					output.normals_.emplace_back(normal.cast<double>());
					const float temp = halfSizeMin/halfSize;
					output.colors_.emplace_back(normal.cast<double>());
				}
			}
		}

		void voxelResampling(open3d::PointCloud &output, const std::vector<DepthImageSource> &imageSource,
		                     const open3d::PointCloud &inputCloud, const float minHalfSize, const float maxHalfSize)
		{
			output.points_.clear();
			output.colors_.clear();
			output.normals_.clear();
			const Eigen::Vector3f minBound = inputCloud.GetMinBound().cast<float>();
			const Eigen::Vector3f maxBound = inputCloud.GetMaxBound().cast<float>();
			for (float x = minBound.x() + maxHalfSize; x < maxBound.x(); x += maxHalfSize * 2.f)
			{
				for (float y = minBound.y() + maxHalfSize; y < maxBound.y(); y += maxHalfSize * 2.f)
				{
					for (float z = minBound.z() + maxHalfSize; z < maxBound.z(); z += maxHalfSize * 2.f)
					{
						const auto nextFixpoint = Eigen::Vector3f(x, y, z);
						doBuildVoxelResampling(output, nextFixpoint, maxHalfSize, minHalfSize, sigma_max, imageSource);
					}
				}
			}
		}

#pragma clang diagnostic pop
	}
}