//
// Created by netlabs on 21.01.19.
//

#include <Eigen/LU>

#include <Core/Utility/Console.h>
#include <Eigen/Dense>

#include "artekmed/PointCloudProcessing.h"
#include "artekmed/PointProcessing/RegionGrowing.h"

/* Given a point in 3D space, compute the corresponding pixel coordinates in an image with no distortion assumed */
void project_point_to_pixel(Eigen::Vector2f& pixel, const Eigen::Matrix3f& intrin, const Eigen::Vector3f& point) {

    float x = point(0) / point(2), y = point(1) / point(2);

    pixel(0) = x * intrin(0,0) + intrin(0,2);
    pixel(1) = y * intrin(1,1) + intrin(1,2);
}

// Matrix 4x4
//00 01 02 03
//10 11 12 13
//20 21 22 23
//30 31 32 33
//Column Major: 00 10 20 30 01 11 21 31 ...
// Eigen accessors vec(row, col)

///* Transform 3D coordinates relative to one sensor to 3D coordinates relative to another viewpoint */
void transform_point_to_point(Eigen::Vector3f& to_point, const Eigen::Matrix4f& extrin, const Eigen::Vector3f& from_point)
{

    to_point(0) = extrin(0,0) * from_point(0) + extrin(0,1) * from_point(1) + extrin(0,2) * from_point(2) + extrin(0, 3);
    to_point(1) = extrin(1,0) * from_point(0) + extrin(1,1) * from_point(1) + extrin(1,2) * from_point(2) + extrin(1, 3);
    to_point(2) = extrin(2,0) * from_point(0) + extrin(2,1) * from_point(1) + extrin(2,2) * from_point(2) + extrin(2, 3);
//    to_point[0] = extrin->rotation[0] * from_point[0] + extrin->rotation[3] * from_point[1] + extrin->rotation[6] * from_point[2] + extrin->translation[0];
//    to_point[1] = extrin->rotation[1] * from_point[0] + extrin->rotation[4] * from_point[1] + extrin->rotation[7] * from_point[2] + extrin->translation[1];
//    to_point[2] = extrin->rotation[2] * from_point[0] + extrin->rotation[5] * from_point[1] + extrin->rotation[8] * from_point[2] + extrin->translation[2];
}

/* Given pixel coordinates and depth in an image with no distortion or inverse distortion coefficients, compute the corresponding point in 3D space relative to the same camera */
void deproject_pixel_to_point(Eigen::Vector3f& point, const Eigen::Matrix3f& intrin, const Eigen::Vector2f& pixel, float depth)
{

    float x = (pixel(0) - intrin(0,2)) / intrin(0,0);
    float y = (pixel(1) - intrin(1,2)) / intrin(1,1);
    point(0) = depth * x;
    point(1) = depth * y;
    point(2) = depth;
}

//Maximum Threshold to consider that the point is even in this depth image, otherwise it is seen as an occluded point
constexpr float zEpslion = 0.05f;
///*@return true, if point can be seen in this image and is not occluded
bool project_point_to_pixel(Eigen::Vector2f & pixel, const Eigen::Matrix3f& intrin, const Eigen::Vector3f& point, const cv::Mat* depth_image){
	float x = point(0)/point(2);
	float y= point(1)/point(2);

	x= x*intrin(0,0)+intrin(0,2);
	y = y*intrin(1,1)+intrin(1,2);
	if(x < 0 || x > depth_image->cols || y < 0 || y>depth_image->rows){
		//We are outside this image Space
		return false;
	}
	float depth = depth_image->at<float>(x,y);
	if(std::abs(depth-point(2))>zEpslion){
		//Z coordinate does not fit into the depth value: point was occluded
		return false;
	}
	pixel(0) = x;
	pixel(1)=y;
	return true;
}

// build pointcloud from depth/color image using image intrinsics (assume depth==color camera)
void buildColoredPointCloud(
        const cv::Mat& depth_img_rect,
        const cv::Mat& color_img_rect,
        const Eigen::Matrix3f& intr_rect_ir,
        open3d::PointCloud& cloud,
        float depth_scale_factor)
{
    unsigned int w = depth_img_rect.cols;
    unsigned int h = depth_img_rect.rows;

    unsigned int num_valid_pixels = w*h;

    auto &points = cloud.points_;
    auto &colors = cloud.colors_;
    points.resize(num_valid_pixels);
    colors.resize(num_valid_pixels);

// currently we're not compiling with openmp (needs cmake changes and libopenmp dependency)
#pragma omp parallel for schedule(dynamic)
    for (int depth_y = 0; depth_y < h; ++depth_y) {
        int depth_pixel_index = depth_y * w;
        for (int depth_x = 0; depth_x < w; ++depth_x, ++depth_pixel_index) {

            auto &pt = points[depth_pixel_index];

            float z = depth_img_rect.at<float>(depth_y, depth_x);
            // Skip over depth pixels with the value of zero, we have no depth data so we will not write anything into our aligned images
            if ((z != 0) && (!isnan(z)))
            {
                float depth = z * depth_scale_factor;

                // Map the top-left corner of the depth pixel onto the other image
                Eigen::Vector2f depth_pixel(depth_x, depth_y);
                Eigen::Vector3f depth_point;

                deproject_pixel_to_point(depth_point, intr_rect_ir, depth_pixel, depth);

                // store pixel location
                pt(0) = -depth_point(0);
                pt(1) = -depth_point(1);
                pt(2) = -depth_point(2);

                cv::Vec4b pixel = color_img_rect.at<cv::Vec4b>(depth_y, depth_x);
                colors[depth_pixel_index] = Eigen::Vector3d(pixel.val[2], pixel.val[1], pixel.val[0]) / 255.;
            } else {
                pt(0) = pt(1) = pt(2) = 0.;
            }
        }
    }
    //[Michael Wechner] Temporary... seems like the best point to insert my processing tests..
    std::vector<artekmed::pointcloud::DepthImageSource> depth_images = {};
    depth_images.emplace_back();
    cloud = artekmed::pointcloud::regionGrowingResample(cloud,depth_images);
}

// build pointcloud from depth/color image using depth/image intrinsics and depth2color transform
void buildColoredPointCloud(
        const cv::Mat& depth_img_rect,
        const cv::Mat& color_img_rect,
        const Eigen::Matrix3f& intr_rect_ir,
        const Eigen::Matrix3f& intr_rect_rgb,
        const Eigen::Matrix4f& depth2color_tf,
        open3d::PointCloud& cloud,
        float depth_scale_factor)
{
    unsigned int w = depth_img_rect.cols;
    unsigned int h = depth_img_rect.rows;

    unsigned int w_rgb = color_img_rect.cols;
    unsigned int h_rgb = color_img_rect.rows;

    unsigned int num_valid_pixels = w*h;

    auto &points = cloud.points_;
    auto &colors = cloud.colors_;
    points.resize(num_valid_pixels);
    colors.resize(num_valid_pixels);

// currently we're not compiling with openmp (needs cmake changes and libopenmp dependency)
#pragma omp parallel for schedule(dynamic)
    for (int depth_y = 0; depth_y < h; ++depth_y) {
        int depth_pixel_index = depth_y * w;
        for (int depth_x = 0; depth_x < w; ++depth_x, ++depth_pixel_index)
        {

            auto &pt = points[depth_pixel_index];

            uint16_t z = depth_img_rect.at<uint16_t>(depth_y, depth_x);
            // Skip over depth pixels with the value of zero, we have no depth data so we will not write anything into our aligned images
            if (z != 0)
            {
                float depth = z * depth_scale_factor;

                // Map the top-left corner of the depth pixel onto the other image
                Eigen::Vector2f depth_pixel(depth_x, depth_y);
                Eigen::Vector2f other_pixel;
                Eigen::Vector3f depth_point, other_point;

                deproject_pixel_to_point(depth_point, intr_rect_ir, depth_pixel, depth);

                // OpenGL like coordinate system
                // store pixel location
                pt(0) = -depth_point(0);
                pt(1) = -depth_point(1);
                pt(2) = -depth_point(2);

                // now transorm into rgb camera coordinates
                transform_point_to_point(other_point, depth2color_tf, depth_point);
//                other_point = (tf.rotation() * depth_point) + tf.translation();

                project_point_to_pixel(other_pixel, intr_rect_rgb, other_point);

                const int other_x = static_cast<int>(other_pixel(0));
                const int other_y = static_cast<int>(other_pixel(1));

                if (other_x < 0 || other_y < 0 || other_x >= w_rgb || other_y >= h_rgb)
                    continue;

                // assumes BGR
                cv::Vec3b pixel = color_img_rect.at<cv::Vec3b>(other_y, other_x);
                colors[depth_pixel_index] = Eigen::Vector3d(pixel.val[2], pixel.val[1], pixel.val[0]) / 255.;
            } else {
                pt(0) = pt(1) = pt(2) = 0.;
            }
        }
    }
}