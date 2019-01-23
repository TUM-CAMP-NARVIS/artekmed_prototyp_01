//
// Created by netlabs on 21.01.19.
//

#ifndef ARTEKMED_P1_POINTCLOUDPROCESSING_H
#define ARTEKMED_P1_POINTCLOUDPROCESSING_H

#include <boost/shared_ptr.hpp>

#include <Eigen/Core>
#include <Core/Geometry/PointCloud.h>
#include <opencv2/opencv.hpp>


void buildPointCloud(
        const cv::Mat& depth_img_rect,
        const Eigen::Matrix3d& intr_rect_ir,
        open3d::PointCloud& cloud,
        double depth_scale_factor=1.0);

//void buildPointCloud(
//        const cv::Mat& depth_img_rect_reg,
//        const cv::Mat& rgb_img_rect,
//        const cv::Mat& intr_rect_rgb,
//        open3d::PointCloud& cloud);
//
//void buildRegisteredDepthImage(
//        const cv::Mat& intr_rect_ir,
//        const cv::Mat& intr_rect_rgb,
//        const cv::Mat& ir2rgb,
//        const cv::Mat& depth_img_rect,
//        cv::Mat& depth_img_rect_reg);



#endif //ARTEKMED_P1_POINTCLOUDPROCESSING_H
