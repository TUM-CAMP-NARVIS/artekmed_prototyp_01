//
// Created by narvis on 14.05.19.
//

#ifndef ARTEKMED_P1_COMMON_H
#define ARTEKMED_P1_COMMON_H

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <algorithm>

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
		Eigen::Matrix3f intrinsics;
		Eigen::Matrix4f extrinsics;
		float depthScaleFactor;

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
	/*
	void swap(DepthImageSource& a, DepthImageSource&b){
		std::swap(a.depthImage,b.depthImage);
		std::swap(a.intrinsics,b.intrinsics);
		std::swap(a.extrinsics,b.extrinsics);
		std::swap(a.depthScaleFactor,b.depthScaleFactor);
	}*/

	///*@return true, if point can be seen in this image and is not occluded
	bool project_point_to_pixel(
		Eigen::Vector2i &pixel,
		const Eigen::Vector3f &point,
		const DepthImageSource &depth_image,
		const float radius = FLT_MAX);

	// Most normal estimations return the correct direction, but the normal can be flipped (i.e. does not point outside)
	// By knowing that cameras are always outside the object , we can find the correct direction.
	inline void alignNormal(
		Eigen::Vector3f &normal,
		const Eigen::Vector3f &myPos,
		const Eigen::Vector3f &cameraWorldPos
	)
	{
		const auto b = cameraWorldPos - myPos;
		if (normal.dot(b) > 0)
		{
			normal *= -1;
		}
	}

	Eigen::Vector3f getCameraPositionFromExtrinsics(const Eigen::Matrix4f& cameraExtrinsics);

	Eigen::Vector3f getCameraDirectionFromExtrinsics(const Eigen::Matrix4f& cameraExtrinsics);

	/*
		Computes the Weight loosely inspired from Keller's method in Real-Time 3D Reconstruction in Dynamic Scenes using
		Point-based Fusion. We use a continious weighting function instead of a discrete reject/keep and additionaly
		consider the offset from the image center
	*/
	float cameraQualityWeight(const float depth,
		const Eigen::Vector2i imageCoordinates,
		const Eigen::Vector3f & pointNormal,
		const Eigen::Matrix4f & cameraExtrinsics);
}

#endif //ARTEKMED_P1_COMMON_H
