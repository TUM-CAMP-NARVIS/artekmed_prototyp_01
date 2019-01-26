//
// Created by netlabs on 21.01.19.
//

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
                pt(2) = z_metric;

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
        const Eigen::Matrix3d& intr_rect_co,
        const Eigen::Matrix4d& depth2color_tf,
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
            uint16_t z = depth_img_rect.at<uint16_t>(v, u);
//            cv::Vec4b pixel = color_img_rect.at<cv::Vec4b>(v, u);

            auto& pt = points[v*w + u];

            if (z != 0)
            {
                double z_metric = z * depth_scale_factor;

                pt(0) = z_metric * ((u - cx) * fx_inv);
                pt(1) = z_metric * ((v - cy) * fy_inv);
                pt(2) = z_metric;

//                colors[v*w + u] = Eigen::Vector3d(pixel.val[2], pixel.val[1], pixel.val[0]) / 255.;
            }
            else
            {
                pt(0) = pt(1) = pt(2) = 0.; //std::numeric_limits<float>::quiet_NaN();
            }
        }
}


/**
void buildPointCloud(
        const cv::Mat& depth_img_rect_reg,
        const cv::Mat& rgb_img_rect,
        const cv::Mat& intr_rect_rgb,
        open3d::PointCloud& cloud)
{
    int w = rgb_img_rect.cols;
    int h = rgb_img_rect.rows;

    double cx = intr_rect_rgb.at<double>(0,2);
    double cy = intr_rect_rgb.at<double>(1,2);
    double fx_inv = 1.0 / intr_rect_rgb.at<double>(0,0);
    double fy_inv = 1.0 / intr_rect_rgb.at<double>(1,1);

    cloud.resize(w*h);

    for (int u = 0; u < w; ++u)
        for (int v = 0; v < h; ++v)
        {
            uint16_t z = depth_img_rect_reg.at<uint16_t>(v, u);
            const cv::Vec3b& c = rgb_img_rect.at<cv::Vec3b>(v, u);

            PointT& pt = cloud.points[v*w + u];

            if (z != 0)
            {
                double z_metric = z * 0.001;

                pt.x = z_metric * ((u - cx) * fx_inv);
                pt.y = z_metric * ((v - cy) * fy_inv);
                pt.z = z_metric;

                pt.r = c[2];
                pt.g = c[1];
                pt.b = c[0];
            }
            else
            {
                pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
            }
        }

    cloud.width = w;
    cloud.height = h;
    cloud.is_dense = true;
}

void buildRegisteredDepthImage(
        const cv::Mat& intr_rect_ir,
        const cv::Mat& intr_rect_rgb,
        const cv::Mat& ir2rgb,
        const cv::Mat& depth_img_rect,
        cv::Mat& depth_img_rect_reg)
{
    int w = depth_img_rect.cols;
    int h = depth_img_rect.rows;

    depth_img_rect_reg = cv::Mat::zeros(h, w, CV_16UC1);

    cv::Mat intr_rect_ir_inv = intr_rect_ir.inv();

    // Eigen intr_rect_rgb (3x3)
    Eigen::Matrix<double, 3, 3> intr_rect_rgb_eigen;
    for (int u = 0; u < 3; ++u)
        for (int v = 0; v < 3; ++v)
            intr_rect_rgb_eigen(v,u) =  intr_rect_rgb.at<double>(v, u);

    // Eigen rgb2ir_eigen (3x4)
    Eigen::Matrix<double, 3, 4> ir2rgb_eigen;
    for (int u = 0; u < 4; ++u)
        for (int v = 0; v < 3; ++v)
            ir2rgb_eigen(v,u) =  ir2rgb.at<double>(v, u);

    // Eigen intr_rect_ir_inv (4x4)
    Eigen::Matrix4d intr_rect_ir_inv_eigen;
    for (int v = 0; v < 3; ++v)
        for (int u = 0; u < 3; ++u)
            intr_rect_ir_inv_eigen(v,u) = intr_rect_ir_inv.at<double>(v,u);

    intr_rect_ir_inv_eigen(0, 3) = 0;
    intr_rect_ir_inv_eigen(1, 3) = 0;
    intr_rect_ir_inv_eigen(2, 3) = 0;
    intr_rect_ir_inv_eigen(3, 0) = 0;
    intr_rect_ir_inv_eigen(3, 1) = 0;
    intr_rect_ir_inv_eigen(3, 2) = 0;
    intr_rect_ir_inv_eigen(3, 3) = 1;

    // multiply into single (3x4) matrix
    Eigen::Matrix<double, 3, 4> H_eigen =
            intr_rect_rgb_eigen * (ir2rgb_eigen * intr_rect_ir_inv_eigen);

    // *** reproject

    Eigen::Vector3d p_rgb;
    Eigen::Vector4d p_depth;

    for (int v = 0; v < h; ++v)
        for (int u = 0; u < w; ++u)
        {
            uint16_t z = depth_img_rect.at<uint16_t>(v,u);

            if (z != 0)
            {
                p_depth(0,0) = u * z;
                p_depth(1,0) = v * z;
                p_depth(2,0) = z;
                p_depth(3,0) = 1.0;
                p_rgb = H_eigen * p_depth;

                double px = p_rgb(0,0);
                double py = p_rgb(1,0);
                double pz = p_rgb(2,0);

                int qu = (int)(px / pz);
                int qv = (int)(py / pz);

                // skip outside of image
                if (qu < 0 || qu >= w || qv < 0 || qv >= h) continue;

                uint16_t& val = depth_img_rect_reg.at<uint16_t>(qv, qu);

                // z buffering
                if (val == 0 || val > pz) val = pz;
            }
        }
}
**/