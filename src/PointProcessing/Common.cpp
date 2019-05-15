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
		Eigen::Vector2i &pixel,
		const Eigen::Vector3f &point,
		const DepthImageSource &depth_image,
		const float radius)
	{
		//NaN bug
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
		const auto zEpslion = radius * std::max(depth_image.intrinsics(0, 0), depth_image.intrinsics(1, 1)) / depth *
		                      depth_image.depthScaleFactor;
		if (std::abs(depth - point(2)) > zEpslion)
		{
			//Z coordinate does not fit into the depth value: point was occluded
			return false;
		}
		pixel(0) = (int) x;
		pixel(1) = (int) y;
		return true;
	}
}