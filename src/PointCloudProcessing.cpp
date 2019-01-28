//
// Created by netlabs on 21.01.19.
//

#include <Eigen/LU>

#include <Core/Utility/Console.h>
#include <Eigen/Dense>

#include "artekmed/PointCloudProcessing.h"


/* Given a point in 3D space, compute the corresponding pixel coordinates in an image with no distortion assumed */
void project_point_to_pixel(Eigen::Vector2d& pixel, const Eigen::Matrix3d& intrin, const Eigen::Vector3d& point) {

    double x = point(0) / point(2), y = point(1) / point(2);

    pixel[0] = x * intrin(0,0) + intrin(0,2);
    pixel[1] = y * intrin(1,1) + intrin(1,2);
}

/* Given pixel coordinates and depth in an image with no distortion or inverse distortion coefficients, compute the corresponding point in 3D space relative to the same camera */
void deproject_pixel_to_point(Eigen::Vector3d& point, const Eigen::Matrix3d& intrin, const Eigen::Vector2d& pixel, float depth)
{

    double x = (pixel(0) - intrin(0,2)) / intrin(0,0);
    double y = (pixel(1) - intrin(1,2)) / intrin(1,1);
    point(0) = depth * x;
    point(1) = depth * y;
    point(2) = depth;
}

// build pointcloud from depth/color image using image intrinsics (assume depth==color camera)
void buildColoredPointCloud(
        const cv::Mat& depth_img_rect,
        const cv::Mat& color_img_rect,
        const Eigen::Matrix3d& intr_rect_ir,
        open3d::PointCloud& cloud,
        double depth_scale_factor)
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
                Eigen::Vector2d depth_pixel(depth_x, depth_y);
                Eigen::Vector3d depth_point;

                deproject_pixel_to_point(depth_point, intr_rect_ir, depth_pixel, depth);

                // store pixel location
                pt(0) = depth_point(0);
                pt(1) = depth_point(1);
                pt(2) = -depth_point(2);

                cv::Vec4b pixel = color_img_rect.at<cv::Vec4b>(depth_y, depth_x);
                colors[depth_pixel_index] = Eigen::Vector3d(pixel.val[2], pixel.val[1], pixel.val[0]) / 255.;
            } else {
                pt(0) = pt(1) = pt(2) = 0.;
            }
        }
    }
}

// build pointcloud from depth/color image using depth/image intrinsics and depth2color transform
void buildColoredPointCloud(
        const cv::Mat& depth_img_rect,
        const cv::Mat& color_img_rect,
        const Eigen::Matrix3d& intr_rect_ir,
        const Eigen::Matrix3d& intr_rect_rgb,
        const Eigen::Matrix4d& depth2color_tf,
        open3d::PointCloud& cloud,
        double depth_scale_factor)
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

    // tf - Depth2Color Transform
    Eigen::Transform<double,3,Eigen::Affine> tf(depth2color_tf);

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
                Eigen::Vector2d depth_pixel(depth_x, depth_y);
                Eigen::Vector2d other_pixel;
                Eigen::Vector3d depth_point, other_point;

                deproject_pixel_to_point(depth_point, intr_rect_ir, depth_pixel, depth);

                // store pixel location
                pt(0) = depth_point(0);
                pt(1) = depth_point(1);
                pt(2) = -depth_point(2);

                // now transorm into rgb camera coordinates
                other_point = (tf.rotation() * depth_point) + tf.translation();

                project_point_to_pixel(other_pixel, intr_rect_rgb, other_point);

                const int other_x = static_cast<int>(other_pixel(0));
                const int other_y = static_cast<int>(other_pixel(1));

                if (other_x < 0 || other_y < 0 || other_x >= w_rgb || other_y >= h_rgb)
                    continue;

                cv::Vec4b pixel = color_img_rect.at<cv::Vec4b>(other_y, other_x);
                // assumes BGRA
                colors[depth_pixel_index] = Eigen::Vector3d(pixel.val[2], pixel.val[1], pixel.val[0]) / 255.;
            } else {
                pt(0) = pt(1) = pt(2) = 0.;
            }
        }
    }
}