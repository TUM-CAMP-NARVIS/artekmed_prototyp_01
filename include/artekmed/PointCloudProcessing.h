//
// Created by netlabs on 21.01.19.
//

#ifndef ARTEKMED_P1_POINTCLOUDPROCESSING_H
#define ARTEKMED_P1_POINTCLOUDPROCESSING_H

#include <boost/shared_ptr.hpp>

#include <Eigen/Core>
#include <Core/Geometry/PointCloud.h>
#include <opencv2/opencv.hpp>


void buildColoredPointCloud(
        const cv::Mat& depth_img_rect,
        const cv::Mat& color_img_rect,
        const Eigen::Matrix3f& intr_rect_ir,
        open3d::PointCloud& cloud,
        float depth_scale_factor=1.0);

void buildColoredPointCloud(
        const cv::Mat& depth_img_rect,
        const cv::Mat& color_img_rect,
        const Eigen::Matrix3f& intr_rect_ir,
        const Eigen::Matrix3f& intr_rect_rgb,
        const Eigen::Matrix4f& depth2color_tf,
        open3d::PointCloud& cloud,
        float depth_scale_factor=1.0);


#endif //ARTEKMED_P1_POINTCLOUDPROCESSING_H
