//
// Created by narvis on 14.05.19.
//

#ifndef ARTEKMED_P1_COMMON_H
#define ARTEKMED_P1_COMMON_H

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

//Utility Macros

#define CHECK_BIT(field, pos) (((field)&(1<<(pos)))!=0)

namespace artekmed
{
	class DepthImageSource;

	//We get an open3d::PointCloud as input data and indices/stable iterators are hard to find there, we
	// instead use the image coordinates to identify a single Pixel
	struct InputSampleCoordinate
	{
		int x;
		int y;
		uint8_t imageIndex;

		InputSampleCoordinate(const int _x, const int _y, const uint8_t _imageIndex) :
			x(_x),
			y(_y),
			imageIndex(_imageIndex)
		{}

		InputSampleCoordinate(const InputSampleCoordinate &) = default;

		InputSampleCoordinate(InputSampleCoordinate &&) = default;

		bool operator==(const InputSampleCoordinate &a) const
		{
			return x == a.x && y == a.y && imageIndex == a.imageIndex;
		}

		InputSampleCoordinate &operator=(const InputSampleCoordinate &) = default;

		struct hash
		{
			size_t operator()(const InputSampleCoordinate &i) const
			{
				return std::hash<int>()(i.x) + std::hash<int>()(i.y) + std::hash<uint8_t>()(i.imageIndex);
			}
		};

		Eigen::Vector3f getPointIn3D(const std::vector<DepthImageSource> &inputImages) const;
	};


	struct DepthImageSource
	{
		const cv::Mat *depthImage;
		const Eigen::Matrix3f intrinsics;
		const Eigen::Matrix4f extrinsics;
		const float depthScaleFactor;

		DepthImageSource(const cv::Mat *img,
		                 const Eigen::Matrix3f &intrinsicMat,
		                 const Eigen::Matrix4f &extrinsicMat,
		                 const float depthScaleFactor) :
			intrinsics(intrinsicMat),
			extrinsics(extrinsicMat),
			depthImage(img),
			depthScaleFactor(depthScaleFactor)
		{}
	};

	///*@return true, if point can be seen in this image and is not occluded
	bool project_point_to_pixel(
		Eigen::Vector2i &pixel,
		const Eigen::Vector3f &point,
		const DepthImageSource &depth_image,
		const float radius);

	// Most normal estimations return the correct direction, but the normal can be flipped (i.e. does not point outside)
	// By knowing that cameras are always outside the object , we can find the correct direction.
	inline void alignNormal(
		Eigen::Vector3f &normal,
		const Eigen::Vector3f &myPos,
		const Eigen::Vector3f &cameraWorldPos
	)
	{
		const auto b = cameraWorldPos - myPos;
		if (normal.dot(b) < 0)
		{
			normal *= -1;
		}
	}

	template<class VectorItType>
	void mlsNormalEstimation(
		std::vector<Eigen::Vector3f>& inputPoints,
		Eigen::Vector3f &test,
		const size_t mainPointIndex,
		const uint8_t degree = 1,
		const Eigen::Vector3f & initialEstimate = {1,0,0},
		const Eigen::Vector3f & centroid = {-1,0,0})
	{

	}
}

#endif //ARTEKMED_P1_COMMON_H
