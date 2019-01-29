//
// Created by Ulrich Eck on 15.03.18.
//

// implementation of UbitrackPointCloudConnector

#include <functional>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include "artekmed/UbitrackPointCloudConnector.h"
#include "artekmed/EigenWrapper.h"
#include "artekmed/PointCloudProcessing.h"

#include <boost/bind.hpp>

#include <log4cpp/Category.hh>
static log4cpp::Category& logger(log4cpp::Category::getInstance("ArtekmedP1.UbitrackPointCloudConnector"));


UbitrackPointCloudConnector::UbitrackPointCloudConnector(const std::string& _components_path)
        : UbitrackBaseConnector(_components_path)
{

}

void UbitrackPointCloudConnector::add_cameras() {

    m_cameras.emplace_back(std::make_shared<artekmed::RGBDCameraConnector>("camera01"));
    m_cameras.emplace_back(std::make_shared<artekmed::RGBDCameraConnector>("camera02"));
    m_cameras.emplace_back(std::make_shared<artekmed::RGBDCameraConnector>("camera03"));
}

//
//
//bool UbitrackPointCloudConnector::camera01_get_pose(Ubitrack::Measurement::Timestamp ts, Eigen::Matrix4d &transform) {
//    try {
//        Ubitrack::Measurement::Pose cam_pose = m_pullsink_camera01_pose->get(ts);
//
//        if (cam_pose) {
//            Eigen::Quaternion<double> rotation = Ubitrack::Math::Wrapper::EigenQuaternionCast().to_eigen(cam_pose->rotation());
//            Eigen::Vector3d position = Ubitrack::Math::Wrapper::EigenVectorCast().to_eigen<double, 3>(cam_pose->translation());
//
//            Eigen::Matrix4d t;
//            t.setIdentity();
//            t.topLeftCorner<3,3>() = rotation.toRotationMatrix();
//            t.topRightCorner<3,1>() = position;
//
//            transform = t;
//
//        } else {
//            LOG4CPP_WARN(logger, "Error getting camera1 pose");
//            return false;
//        }
//
//    } catch (std::exception &e) {
//        LOG4CPP_WARN(logger, "error retrieving pose for camera01: " << e.what());
//        return false;
//    }
//    return true;
//}
//
//bool UbitrackPointCloudConnector::camera01_get_pointcloud(Ubitrack::Measurement::Timestamp ts, std::shared_ptr<open3d::PointCloud>& cloud)
//{
//    if (!have_camera01()) {
//        return false;
//    }
//
//    // we need locking here to prevent concurrent access to m_current_camera01_image (when receiving new frame)
//    std::unique_lock<std::mutex> ul( m_textureAccessMutex );
//
//    try {
//        Ubitrack::Measurement::ImageMeasurement depth_image = m_pullsink_camera01_depth->get(ts);
//        Ubitrack::Measurement::CameraIntrinsics image_model = m_pullsink_camera01_image_model->get(ts);
//        Ubitrack::Measurement::Pose camera_pose = m_pullsink_camera01_pose->get(ts);
//
//        Eigen::Matrix4d origin_tf;
//        {
//            Eigen::Quaternion<double> rotation = Ubitrack::Math::Wrapper::EigenQuaternionCast().to_eigen(camera_pose->rotation());
//            Eigen::Vector3d position = Ubitrack::Math::Wrapper::EigenVectorCast().to_eigen<double, 3>(camera_pose->translation());
//
//            origin_tf.setIdentity();
//            origin_tf.topLeftCorner<3,3>() = rotation.toRotationMatrix();
//            origin_tf.topRightCorner<3,1>() = position;
//        }
//
//        auto depth_img = depth_image->Mat();
//        auto color_img = m_current_camera01_image->Mat();
//
//        auto num_pixels_color = m_current_camera01_image->width() * m_current_camera01_image->height();
//        auto num_pixels_depth = depth_image->width() * depth_image->height();
//
//        if (num_pixels_color != num_pixels_depth) {
//            LOG4CPP_ERROR(logger, "Color and Depth image for ZED Camera must have the same size!")
//            return false;
//        }
//
//        if ((depth_image->pixelFormat() != Ubitrack::Vision::Image::DEPTH) || (depth_image->bitsPerPixel() != 32)) {
//            LOG4CPP_ERROR(logger, "unsupported depth format - need DEPTH / UINT16!")
//            return false;
//        }
//
//        Eigen::Matrix3d intrinsics = Ubitrack::Math::Wrapper::EigenIntrinsicMatrixCast().to_eigen(image_model->matrix);
//
//        buildPointCloudZED(depth_img, color_img, intrinsics, *cloud, 0.001);
//
//        // transform into origin coordinates
////        cloud->Transform(origin_tf);
//
//    } catch (std::exception &e) {
//        LOG4CPP_WARN(logger, "error retrieving measurement for camera01: " << e.what());
//        return false;
//    }
//
//    return true;
//}
//
//bool UbitrackPointCloudConnector::camera02_get_pose(Ubitrack::Measurement::Timestamp ts, Eigen::Matrix4d &transform) {
//    try {
//        Ubitrack::Measurement::Pose cam_pose = m_pullsink_camera02_pose->get(ts);
//
//        if (cam_pose) {
//            Eigen::Quaternion<double> rotation = Ubitrack::Math::Wrapper::EigenQuaternionCast().to_eigen(cam_pose->rotation());
//            Eigen::Vector3d position = Ubitrack::Math::Wrapper::EigenVectorCast().to_eigen<double, 3>(cam_pose->translation());
//
//            Eigen::Matrix4d t;
//            t.setIdentity();
//            t.topLeftCorner<3,3>() = rotation.toRotationMatrix();
//            t.topRightCorner<3,1>() = position;
//
//            transform = t;
//
//        } else {
//            LOG4CPP_WARN(logger, "Error getting camera2 pose");
//            return false;
//        }
//
//    } catch (std::exception &e) {
//        LOG4CPP_WARN(logger, "error retrieving pose for camera02: " << e.what());
//        return false;
//    }
//    return true;
//}
//
//bool UbitrackPointCloudConnector::camera02_get_pointcloud(Ubitrack::Measurement::Timestamp ts, std::shared_ptr<open3d::PointCloud>& cloud)
//{
//    if (!have_camera02()) {
//        return false;
//    }
//    // we need locking here to prevent concurrent access to m_current_camera01_image (when receiving new frame)
//    std::unique_lock<std::mutex> ul( m_textureAccessMutex );
//
//    try {
//        Ubitrack::Measurement::ImageMeasurement color_image = m_pullsink_camera02_image->get(ts);
//        Ubitrack::Measurement::ImageMeasurement depth_image = m_pullsink_camera02_depth->get(ts);
//        Ubitrack::Measurement::CameraIntrinsics image_model = m_pullsink_camera02_image_model->get(ts);
//        Ubitrack::Measurement::CameraIntrinsics depth_model = m_pullsink_camera02_depth_model->get(ts);
//        Ubitrack::Measurement::Pose camera_pose = m_pullsink_camera02_pose->get(ts);
//        Ubitrack::Measurement::Pose camera_depth2color = m_pullsink_camera02_depth2color->get(ts);
//
//        Eigen::Matrix4d origin_tf;
//        {
//            Eigen::Quaternion<double> rotation = Ubitrack::Math::Wrapper::EigenQuaternionCast().to_eigen(camera_pose->rotation());
//            Eigen::Vector3d position = Ubitrack::Math::Wrapper::EigenVectorCast().to_eigen<double, 3>(camera_pose->translation());
//
//            origin_tf.setIdentity();
//            origin_tf.topLeftCorner<3,3>() = rotation.toRotationMatrix();
//            origin_tf.topRightCorner<3,1>() = position;
//        }
//
//        Eigen::Matrix4d depth2color_tf;
//        {
//            Eigen::Quaternion<double> rotation = Ubitrack::Math::Wrapper::EigenQuaternionCast().to_eigen(camera_depth2color->rotation());
//            Eigen::Vector3d position = Ubitrack::Math::Wrapper::EigenVectorCast().to_eigen<double, 3>(camera_depth2color->translation());
//
//            depth2color_tf.setIdentity();
//            depth2color_tf.topLeftCorner<3,3>() = rotation.toRotationMatrix();
//            depth2color_tf.topRightCorner<3,1>() = position;
//        }
//
//        auto depth_img = depth_image->Mat();
//        auto color_img = color_image->Mat();
//
//        if ((depth_image->pixelFormat() != Ubitrack::Vision::Image::DEPTH) || (depth_image->bitsPerPixel() != 16)) {
//            LOG4CPP_ERROR(logger, "unsupported depth format - need DEPTH / UINT16!")
//            return false;
//        }
//
//        Eigen::Matrix3d intrinsics_color = Ubitrack::Math::Wrapper::EigenIntrinsicMatrixCast().to_eigen(image_model->matrix);
//        Eigen::Matrix3d intrinsics_depth = Ubitrack::Math::Wrapper::EigenIntrinsicMatrixCast().to_eigen(depth_model->matrix);
//
//        // compute pointcloud and colors
//        buildPointCloudRS(depth_img, color_img, intrinsics_depth, intrinsics_color, depth2color_tf, *cloud, 0.001);
//
//        // transform into origin coordinates
//        cloud->Transform(origin_tf);
//
//    } catch (std::exception &e) {
//        LOG4CPP_WARN(logger, "error retrieving measurement for camera02: " << e.what());
//        return false;
//    }
//    return true;
//}
//
//bool UbitrackPointCloudConnector::camera03_get_pose(Ubitrack::Measurement::Timestamp ts, Eigen::Matrix4d &transform) {
//    try {
//        Ubitrack::Measurement::Pose cam_pose = m_pullsink_camera03_pose->get(ts);
//
//        if (cam_pose) {
//            Eigen::Quaternion<double> rotation = Ubitrack::Math::Wrapper::EigenQuaternionCast().to_eigen(cam_pose->rotation());
//            Eigen::Vector3d position = Ubitrack::Math::Wrapper::EigenVectorCast().to_eigen<double, 3>(cam_pose->translation());
//
//            Eigen::Matrix4d t;
//            t.setIdentity();
//            t.topLeftCorner<3,3>() = rotation.toRotationMatrix();
//            t.topRightCorner<3,1>() = position;
//
//            transform = t;
//
//        } else {
//            LOG4CPP_WARN(logger, "Error getting camera3 pose");
//            return false;
//        }
//
//    } catch (std::exception &e) {
//        LOG4CPP_WARN(logger, "error retrieving pose for camera03: " << e.what());
//        return false;
//    }
//    return true;
//}
//
//bool UbitrackPointCloudConnector::camera03_get_pointcloud(Ubitrack::Measurement::Timestamp ts, std::shared_ptr<open3d::PointCloud>& cloud)
//{
//    if (!have_camera03()) {
//        return false;
//    }
//    // we need locking here to prevent concurrent access to m_current_camera01_image (when receiving new frame)
//    std::unique_lock<std::mutex> ul( m_textureAccessMutex );
//
//    try {
//        Ubitrack::Measurement::PositionList vec3_measurement = m_pullsink_camera03_pointcloud->get(ts);
//        Ubitrack::Measurement::ImageMeasurement img_measurement = m_pullsink_camera03_image->get(ts);
//
//        if ((vec3_measurement) && (!vec3_measurement->empty()) && (img_measurement)) {
//
//            size_t num_valid_pixels = vec3_measurement->size();
//
//            auto img = img_measurement->Mat();
//
//            Ubitrack::Vision::Image::PixelFormat fmt = img_measurement->pixelFormat();
//            if ((fmt == Ubitrack::Vision::Image::BGRA) || (fmt == Ubitrack::Vision::Image::RGBA)) {
//
//                auto& points = cloud->points_;
//                auto& colors = cloud->colors_;
//                const auto& pointcloud = *vec3_measurement;
//
//                points.resize(num_valid_pixels);
//                colors.resize(num_valid_pixels);
//
//                int cnt = 0;
//                for (int i = 0; i < img_measurement->height(); i++) {
//                    for (int j = 0; j < img_measurement->width(); j++) {
//                        if (cnt < num_valid_pixels) {
//                            cv::Vec4b pixel = img.at<cv::Vec4b>(i, j);
//                            auto& p = pointcloud.at(cnt);
//                            points[cnt] = Eigen::Vector3d(p(0), p(1), p(2));
//                            if (fmt == Ubitrack::Vision::Image::BGRA) {
//                                colors[cnt++] = Eigen::Vector3d(pixel.val[2], pixel.val[1], pixel.val[0]) / 255.;
//                            } else {
//                                colors[cnt++] = Eigen::Vector3d(pixel.val[0], pixel.val[1], pixel.val[2]) / 255.;
//                            }
//                        }
//                    }
//                }
//            } else  if (fmt == Ubitrack::Vision::Image::LUMINANCE) {
//
//                auto& points = cloud->points_;
//                auto& colors = cloud->colors_;
//                const auto& pointcloud = *vec3_measurement;
//
//                points.resize(num_valid_pixels);
//                colors.resize(num_valid_pixels);
//
//                int cnt = 0;
//                for (int i = 0; i < m_current_camera01_image->height(); i++) {
//                    for (int j = 0; j < m_current_camera01_image->width(); j++) {
//                        if (cnt < num_valid_pixels) {
//                            uchar pixel = img.at<uchar>(i, j);
//                            auto& p = pointcloud.at(cnt);
//                            points[cnt] = Eigen::Vector3d(p(0), p(1), p(2));
//                            colors[cnt++] = Eigen::Vector3d(pixel, pixel, pixel) / 255.;
//                        }
//                    }
//                }
//            } else {
//                LOG4CPP_WARN( logger, "unknown image format: " << fmt);
//                return false;
//            }
//        } else {
//            LOG4CPP_WARN( logger, "no pointcloud measurement received");
//            return false;
//        }
//    } catch (std::exception &e) {
//        LOG4CPP_WARN(logger, "error retrieving measurement for camera03: " << e.what());
//        return false;
//    }
//
////    Eigen::Matrix4d transform;
////    if (camera03_get_pose(ts, transform)) {
////        cloud->Transform(transform);
////    }
//
//    return true;
//}


