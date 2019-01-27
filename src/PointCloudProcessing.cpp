//
// Created by netlabs on 21.01.19.
//

#include <Eigen/LU>


#include "artekmed/PointCloudProcessing.h"

void buildPointCloudZED(
        const cv::Mat& depth_img_rect,
        const cv::Mat& color_img_rect,
        const Eigen::Matrix3d& intr_rect_ir,
        open3d::PointCloud& cloud,
        double depth_scale_factor)
{
    unsigned int w = depth_img_rect.cols;
    unsigned int h = depth_img_rect.rows;

    double cx = -intr_rect_ir(0,2);
    double cy = -intr_rect_ir(1,2);
    double fx_inv = 1.0 / intr_rect_ir(0,0);
    double fy_inv = 1.0 / intr_rect_ir(1,1);

    unsigned int num_valid_pixels = w*h;

    auto &points = cloud.points_;
    auto &colors = cloud.colors_;
    points.resize(num_valid_pixels);
    colors.resize(num_valid_pixels);

    for (int u = 0; u < w; ++u)
        for (int v = 0; v < h; ++v)
        {
            float z = depth_img_rect.at<float>(v, u);
            cv::Vec4b pixel = color_img_rect.at<cv::Vec4b>(v, u);

            auto& pt = points[v*w + u];

            if ((z != 0) && (!isnan(z)))
            {
                double z_metric = z * depth_scale_factor;

                pt(0) = z_metric * ((u - cx) * fx_inv);
                pt(1) = z_metric * ((v - cy) * fy_inv);
                pt(2) = -z_metric;

                colors[v*w + u] = Eigen::Vector3d(pixel.val[2], pixel.val[1], pixel.val[0]) / 255.;
            }
            else
            {
                pt(0) = pt(1) = pt(2) = 0.; //std::numeric_limits<float>::quiet_NaN();
            }
        }
}


void buildPointCloudRS(
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


    double cx = -intr_rect_ir(0,2);
    double cy = -intr_rect_ir(1,2);
    double fx_inv = 1.0 / intr_rect_ir(0,0);
    double fy_inv = 1.0 / intr_rect_ir(1,1);

    unsigned int num_valid_pixels = w*h;

    auto &points = cloud.points_;
    auto &colors = cloud.colors_;
    points.resize(num_valid_pixels);
    colors.resize(num_valid_pixels);


    // inverse depth intrinsics
    Eigen::Matrix4d intr_rect_rgb_inv;
    intr_rect_rgb_inv.topLeftCorner<3,3>() = intr_rect_rgb.inverse();
    intr_rect_rgb_inv(0, 3) = 0;
    intr_rect_rgb_inv(1, 3) = 0;
    intr_rect_rgb_inv(2, 3) = 0;
    intr_rect_rgb_inv(3, 0) = 0;
    intr_rect_rgb_inv(3, 1) = 0;
    intr_rect_rgb_inv(3, 2) = 0;
    intr_rect_rgb_inv(3, 3) = 1;

    // Eigen ir2rgb_eigen (3x4)
    Eigen::Matrix4d rgb2ir_tf = depth2color_tf.inverse();
    Eigen::Matrix<double, 3, 4> rgb2ir;
    for (int u = 0; u < 4; ++u)
        for (int v = 0; v < 3; ++v)
            rgb2ir(v,u) =  rgb2ir_tf(v, u);

    // multiply into single (3x4) matrix
    Eigen::Matrix<double, 3, 4> H_eigen =
            intr_rect_ir * (rgb2ir * intr_rect_rgb_inv);

    // *** reproject
    Eigen::Vector3d p_rgb;
    Eigen::Vector4d p_depth;

    for (int v = 0; v < h; ++v)
        for (int u = 0; u < w; ++u)
        {
            uint16_t z = depth_img_rect.at<uint16_t>(v, u);

            auto& pt = points[v*w + u];

            if (z != 0)
            {
                // compute metric position
                double z_metric = z * depth_scale_factor;

                pt(0) = z_metric * ((u - cx) * fx_inv);
                pt(1) = z_metric * ((v - cy) * fy_inv);
                pt(2) = -z_metric;


                // project into colorspace UV coordinates
                p_depth(0) = u * z;
                p_depth(1) = v * z;
                p_depth(2) = z;
                p_depth(3) = 1.0;
                p_rgb = H_eigen * p_depth;

                double px = p_rgb(0,0);
                double py = p_rgb(1,0);
                double pz = p_rgb(2,0);

                int qu = (int)(px / pz);
                int qv = (int)(py / pz);

                // skip outside of image
                if (qu < 0 || qu >= w_rgb || qv < 0 || qv >= h_rgb) continue;

                cv::Vec4b pixel = color_img_rect.at<cv::Vec4b>(qv, qu);
                colors[v*w + u] = Eigen::Vector3d(pixel.val[2], pixel.val[1], pixel.val[0]) / 255.;
            }
            else
            {
                pt(0) = pt(1) = pt(2) = 0.; //std::numeric_limits<float>::quiet_NaN();
            }
        }
//
//
//    for (int u = 0; u < w; ++u)
//        for (int v = 0; v < h; ++v)
//        {
//            uint16_t z = depth_img_rect.at<uint16_t>(v, u);
//
//            auto& pt = points[v*w + u];
//
//            if (z != 0)
//            {
//                double z_metric = z * depth_scale_factor;
//
//                pt(0) = z_metric * ((u - cx) * fx_inv);
//                pt(1) = z_metric * ((v - cy) * fy_inv);
//                pt(2) = -z_metric;
//
//            }
//            else
//            {
//                pt(0) = pt(1) = pt(2) = 0.; //std::numeric_limits<float>::quiet_NaN();
//            }
//        }
}