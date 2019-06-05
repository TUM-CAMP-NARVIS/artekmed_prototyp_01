//
// Created by narvis on 14.05.19.
//
#include "artekmed/PointCloudProcessing.h"
#include "artekmed/PointProcessing/Common.h"



namespace artekmed{
	Eigen::Vector3f InputSampleCoordinate::getPointIn3D(const std::vector<DepthImageSource> &inputImages) const
	{
		Eigen::Vector3f result;
		::deproject_pixel_to_point(
			result,
			inputImages[imageIndex].intrinsics,
			{x, y},
			inputImages[imageIndex].depthImage->at<float>(y, x) * inputImages[imageIndex].depthScaleFactor
		);
		return result;
	}
	//Projection from 3D to 2D.. checks for occlusion and out of image plane points
	bool project_point_to_pixel(
		Eigen::Vector2i& pixel,
		const Eigen::Vector3f& point,
		const DepthImageSource& depth_image,
		const float radius /*= FLT_MAX*/)
	{
		//Can't be too close to the camera
		if (std::abs(point(2)) < 0.0001f)
		{
			return false;
		}
		float x = point(0) / point(2);
		float y = point(1) / point(2);

		x = x * depth_image.intrinsics(0, 0) + depth_image.intrinsics(0, 2);
		y = y * depth_image.intrinsics(1, 1) + depth_image.intrinsics(1, 2);
		if (x < 0 || x > depth_image.depthImage->cols || y < 0 || y > depth_image.depthImage->rows)
		{
			//We are outside this image Space
			return false;
		}
		float depth = depth_image.depthImage->at<float>((int) y, (int) x) * depth_image.depthScaleFactor;
		const auto zEpslion = radius * std::max(depth_image.intrinsics(0, 0), depth_image.intrinsics(1, 1)) / depth * depth_image.depthScaleFactor;
		if (std::abs(depth - point(2)) > zEpslion)
		{
			//Z coordinate does not fit into the depth value: point was occluded
			return false;
		}
		pixel(0) = std::round(x);
		pixel(1) = std::round(y);
		return true;
	}

	Eigen::Vector3f getCameraPositionFromExtrinsics(const Eigen::Matrix4f& cameraExtrinsics)
	{
		return {
			-cameraExtrinsics(0,3),
			-cameraExtrinsics(1,3),
			-cameraExtrinsics(2,3)
		};
	}

	Eigen::Vector3f getCameraDirectionFromExtrinsics(const Eigen::Matrix4f& cameraExtrinsics)
	{
		auto rotationMatrix = cameraExtrinsics.block<3, 3>(0, 0);
		auto direction = rotationMatrix.transpose() * Eigen::Vector3f{ 0,0,1 };
		return direction;
	}

	float cameraQualityWeight(const float depth,
	                          const Eigen::Vector2i imageCoordinates,
	                          const Eigen::Vector3f & pointNormal,
	                          const Eigen::Matrix4f & cameraExtrinsics)
	{
		//Constants
		constexpr float angleRejectLimit = 0.9f; //in cos(alpha)
		constexpr float angleRejectEnvelope = 5.f;
		constexpr float offsetEnvelope = 0.7f;

		const Eigen::Vector3f cameraDirection = getCameraDirectionFromExtrinsics(cameraExtrinsics);
		auto x_imgNDC = (imageCoordinates.x() - cameraExtrinsics(0, 2)) / cameraExtrinsics(0, 2);
		auto y_imgNDC = (imageCoordinates.y() - cameraExtrinsics(1, 2))/ cameraExtrinsics(1, 2);
		auto w = 1 - std::exp((std::abs(cameraDirection.dot(pointNormal)) - angleRejectLimit) * angleRejectEnvelope);
		auto offsetFromMiddlePoint = (std::abs(x_imgNDC) + std::abs(y_imgNDC)) / 2;
		w -= std::exp(offsetFromMiddlePoint*offsetEnvelope)-1;
		return std::min(0.f, w);
	}

}