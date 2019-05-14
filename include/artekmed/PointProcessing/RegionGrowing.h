
#ifndef ARTEKMED_P1_REGIONGROWING_H
#define ARTEKMED_P1_REGIONGROWING_H
//Base Class for Point Cloud to Point Cloud processing
#include <Core/Core.h>
#include <Eigen/Eigen>

#include "artekmed/PointCloudProcessing.h"

namespace artekmed
{
	namespace pointcloud
	{
		class DepthImageSource;
		//We get an open3d::PointCloud as input data and indices/stable iterators are hard to find there, we
		// instead use the image coordinates to identify a single Pixel
		struct InputSampleCoordinate
		{
			int x;
			int y;
			uint8_t imageIndex;

			InputSampleCoordinate(const int _x, const int _y, const uint8_t _imageIndex):
				x(_x),
				y(_y),
				imageIndex(_imageIndex){}
			InputSampleCoordinate(const InputSampleCoordinate&)=default;
			InputSampleCoordinate(InputSampleCoordinate &&) = default;
			bool operator==(const InputSampleCoordinate & a)const
			{
				return x == a.x && y==a.y && imageIndex == a.imageIndex;
			}
			InputSampleCoordinate& operator=(const InputSampleCoordinate&)=default;

			struct hash{
				size_t operator()(const InputSampleCoordinate & i) const{
					return std::hash<int>()(i.x)+std::hash<int>()(i.y)+std::hash<uint8_t>()(i.imageIndex);
				}
			};

			Eigen::Vector3f getPointIn3D(const std::vector<DepthImageSource> & inputImages)const;
		};


		struct DepthImageSource
			{
			const cv::Mat *depthImage;
			const Eigen::Matrix3f intrinsics;
			const Eigen::Matrix4f extrinsics;
			const float depthScaleFactor;

			DepthImageSource(const cv::Mat *img,
											 const Eigen::Matrix3f& intrinsicMat,
											 const Eigen::Matrix4f& extrinsicMat,
											 const float depthScaleFactor) :
				intrinsics(intrinsicMat),
				extrinsics(extrinsicMat),
				depthImage(img),
				depthScaleFactor(depthScaleFactor){}
		};

		open3d::PointCloud regionGrowingResample(
			const std::vector<Eigen::Vector3d> &inputPointCloud,
			const std::vector<DepthImageSource> &depthImages,
			const int seed,
			const size_t numSamplesTarget
		);
		///*@return true, if point can be seen in this image and is not occluded
		bool project_point_to_pixel(
			Eigen::Vector2i & pixel,
			const Eigen::Vector3f& point,
			const DepthImageSource& depth_image);


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
