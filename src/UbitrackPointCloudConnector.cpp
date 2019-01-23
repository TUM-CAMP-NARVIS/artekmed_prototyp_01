//
// Created by Ulrich Eck on 15.03.18.
//

// implementation of UbitrackPointCloudConnector

#include <functional>
#include <Eigen/Geometry>
#include "artekmed/UbitrackPointCloudConnector.h"

#include <boost/bind.hpp>

#include <log4cpp/Category.hh>
static log4cpp::Category& logger(log4cpp::Category::getInstance("ArtekmedP1.UbitrackPointCloudConnector"));


UbitrackPointCloudConnector::UbitrackPointCloudConnector(const std::string& _components_path)
        : UbitrackBaseConnector(_components_path)
{}

bool UbitrackPointCloudConnector::initialize(const std::string& _utql_filename)
{
    if (!UbitrackBaseConnector::initialize(_utql_filename)){
        return false;
    }

    // create sinks/sources
    // Camera1
    try {
        m_pushsink_camera01_image = m_utFacade->componentByName<Ubitrack::Components::ApplicationPushSinkVisionImage>("camera01_image");
    } catch (std::exception &e) {
        LOG4CPP_ERROR(logger, e.what());
        m_pushsink_camera01_image.reset();
    }
    try{
        m_pullsink_camera01_depth = m_utFacade->componentByName<Ubitrack::Components::ApplicationPullSinkVisionImage>("camera01_depth");
    } catch (std::exception &e) {
        LOG4CPP_ERROR(logger, e.what());
        m_pullsink_camera01_depth.reset();
    }
    try {
        m_pullsink_camera01_pointcloud = m_utFacade->componentByName<Ubitrack::Components::ApplicationPullSinkPositionList>("camera01_pointcloud");
    } catch (std::exception &e) {
        LOG4CPP_ERROR(logger, e.what());
        m_pullsink_camera01_pointcloud.reset();
    }
    try{
        m_pullsink_camera01_pose = m_utFacade->componentByName<Ubitrack::Components::ApplicationPullSinkPose>("camera01_pose");
    } catch (std::exception &e) {
        LOG4CPP_ERROR(logger, e.what());
        m_pullsink_camera01_pose.reset();
    }
    try{
        m_pullsink_camera01_image_model = m_utFacade->componentByName<Ubitrack::Components::ApplicationPullSinkCameraIntrinsics>("camera01_image_model");
    } catch (std::exception &e) {
        LOG4CPP_ERROR(logger, e.what());
        m_pullsink_camera01_image_model.reset();
    }


    // Camera2
    try{
        m_pullsink_camera02_image = m_utFacade->componentByName<Ubitrack::Components::ApplicationPullSinkVisionImage>("camera02_image");
    } catch (std::exception &e) {
        LOG4CPP_ERROR(logger, e.what());
        m_pullsink_camera02_image.reset();
    }
    try{
        m_pullsink_camera02_depth = m_utFacade->componentByName<Ubitrack::Components::ApplicationPullSinkVisionImage>("camera02_depth");
    } catch (std::exception &e) {
        LOG4CPP_ERROR(logger, e.what());
        m_pullsink_camera02_depth.reset();
    }
    try{
        m_pullsink_camera02_pointcloud = m_utFacade->componentByName<Ubitrack::Components::ApplicationPullSinkPositionList>("camera02_pointcloud");
    } catch (std::exception &e) {
        LOG4CPP_ERROR(logger, e.what());
        m_pullsink_camera02_pointcloud.reset();
    }
    try{
        m_pullsink_camera02_pose = m_utFacade->componentByName<Ubitrack::Components::ApplicationPullSinkPose>("camera02_pose");
    } catch (std::exception &e) {
        LOG4CPP_ERROR(logger, e.what());
        m_pullsink_camera02_pose.reset();
    }
    try{
        m_pullsink_camera02_image_model = m_utFacade->componentByName<Ubitrack::Components::ApplicationPullSinkCameraIntrinsics>("camera02_image_model");
    } catch (std::exception &e) {
        LOG4CPP_ERROR(logger, e.what());
        m_pullsink_camera02_image_model.reset();
    }
    try{
        m_pullsink_camera02_depth_model = m_utFacade->componentByName<Ubitrack::Components::ApplicationPullSinkCameraIntrinsics>("camera02_depth_model");
    } catch (std::exception &e) {
        LOG4CPP_ERROR(logger, e.what());
        m_pullsink_camera02_depth_model.reset();
    }
    try{
        m_pullsink_camera02_depth2color = m_utFacade->componentByName<Ubitrack::Components::ApplicationPullSinkPose>("camera02_depth2color");
    } catch (std::exception &e) {
        LOG4CPP_ERROR(logger, e.what());
        m_pullsink_camera02_depth2color.reset();
    }


    // Camera3
    try {
        m_pullsink_camera03_image = m_utFacade->componentByName<Ubitrack::Components::ApplicationPullSinkVisionImage>("camera03_image");
    } catch (std::exception &e) {
        LOG4CPP_ERROR(logger, e.what());
        m_pullsink_camera03_image.reset();
    }
    try {
        m_pullsink_camera03_depth = m_utFacade->componentByName<Ubitrack::Components::ApplicationPullSinkVisionImage>("camera03_depth");
    } catch (std::exception &e) {
        LOG4CPP_ERROR(logger, e.what());
        m_pullsink_camera03_depth.reset();
    }
    try {
        m_pullsink_camera03_pointcloud = m_utFacade->componentByName<Ubitrack::Components::ApplicationPullSinkPositionList>("camera03_pointcloud");
    } catch (std::exception &e) {
        LOG4CPP_ERROR(logger, e.what());
        m_pullsink_camera03_pointcloud.reset();
    }
    try {
        m_pullsink_camera03_pose = m_utFacade->componentByName<Ubitrack::Components::ApplicationPullSinkPose>("camera03_pose");
    } catch (std::exception &e) {
        LOG4CPP_ERROR(logger, e.what());
        m_pullsink_camera03_pose.reset();
    }
    try{
        m_pullsink_camera03_image_model = m_utFacade->componentByName<Ubitrack::Components::ApplicationPullSinkCameraIntrinsics>("camera03_image_model");
    } catch (std::exception &e) {
        LOG4CPP_ERROR(logger, e.what());
        m_pullsink_camera03_image_model.reset();
    }
    try{
        m_pullsink_camera03_depth_model = m_utFacade->componentByName<Ubitrack::Components::ApplicationPullSinkCameraIntrinsics>("camera03_depth_model");
    } catch (std::exception &e) {
        LOG4CPP_ERROR(logger, e.what());
        m_pullsink_camera03_depth_model.reset();
    }
    try{
        m_pullsink_camera03_depth2color = m_utFacade->componentByName<Ubitrack::Components::ApplicationPullSinkPose>("camera03_depth2color");
    } catch (std::exception &e) {
        LOG4CPP_ERROR(logger, e.what());
        m_pullsink_camera03_depth2color.reset();
    }

    // Push handler callback
    if (m_pushsink_camera01_image) {
        m_pushsink_camera01_image->setCallback(boost::bind( &UbitrackPointCloudConnector::receive_camera01_image, this, _1 ));
    }
    return true;
}

bool UbitrackPointCloudConnector::teardown()
{
    // need to unregister callback here !!!
    if (m_pushsink_camera01_image) {
        m_pushsink_camera01_image->setCallback(NULL);
    }
    // deallocate sinks to be sure there is no activity before deallocating the facade
    m_pushsink_camera01_image.reset();
    m_pullsink_camera01_depth.reset();
    m_pullsink_camera01_pointcloud.reset();
    m_pullsink_camera01_pose.reset();
    m_pullsink_camera01_image_model.reset();

    m_pullsink_camera02_image.reset();
    m_pullsink_camera02_depth.reset();
    m_pullsink_camera02_pointcloud.reset();
    m_pullsink_camera02_pose.reset();
    m_pullsink_camera02_image_model.reset();
    m_pullsink_camera02_depth_model.reset();

    m_pullsink_camera03_image.reset();
    m_pullsink_camera03_depth.reset();
    m_pullsink_camera03_pointcloud.reset();
    m_pullsink_camera03_pose.reset();
    m_pullsink_camera03_image_model.reset();
    m_pullsink_camera03_depth_model.reset();

    return UbitrackBaseConnector::teardown();
}

bool UbitrackPointCloudConnector::camera01_get_pointcloud(Ubitrack::Measurement::Timestamp ts, std::shared_ptr<open3d::PointCloud>& cloud)
{
    if (!have_camera01()) {
        return false;
    }

    // we need locking here to prevent concurrent access to m_current_camera01_image (when receiving new frame)
    std::unique_lock<std::mutex> ul( m_textureAccessMutex );

    try {
        Ubitrack::Measurement::ImageMeasurement depth_image = m_pullsink_camera01_depth->get(ts);
        Ubitrack::Measurement::PositionList vec3_measurement = m_pullsink_camera01_pointcloud->get(ts);
        Ubitrack::Measurement::ImageMeasurement depth_measurement = m_pullsink_camera01_depth->get(ts);

        auto depth_img = depth_measurement->Mat();
        auto color_img = m_current_camera01_image->Mat();

        auto num_pixels_color = m_current_camera01_image->width() * m_current_camera01_image->height();
        auto num_pixels_depth = depth_measurement->width() * depth_measurement->height();

        if (num_pixels_color != num_pixels_depth) {
            LOG4CPP_ERROR(logger, "Color and Depth image for ZED Camera must have the same size!")
            return false;
        }

        Ubitrack::Measurement::CameraIntrinsics image_model = m_pullsink_camera01_image_model->get(ts);
        Ubitrack::Measurement::Pose camera_pose = m_pullsink_camera01_pose->get(ts);







        if ((vec3_measurement) && (!vec3_measurement->empty())) {

            size_t num_valid_pixels = vec3_measurement->size();

            auto img = m_current_camera01_image->Mat();

            Ubitrack::Vision::Image::PixelFormat fmt = m_current_camera01_image->pixelFormat();
            if ((fmt == Ubitrack::Vision::Image::BGRA) || (fmt == Ubitrack::Vision::Image::RGBA)) {

                auto &points = cloud->points_;
                auto &colors = cloud->colors_;
                const auto &pointcloud = *vec3_measurement;

                points.resize(num_valid_pixels);
                colors.resize(num_valid_pixels);

                int cnt = 0;
                for (int i = 0; i < m_current_camera01_image->height(); i++) {
                    for (int j = 0; j < m_current_camera01_image->width(); j++) {
                        if (cnt < num_valid_pixels) {
                            cv::Vec4b pixel = img.at<cv::Vec4b>(i, j);
                            auto &p = pointcloud.at(cnt);
                            points[cnt] = Eigen::Vector3d(p(0), p(1), p(2));
                            if (fmt == Ubitrack::Vision::Image::BGRA) {
                                colors[cnt++] = Eigen::Vector3d(pixel.val[2], pixel.val[1], pixel.val[0]) / 255.;
                            } else {
                                colors[cnt++] = Eigen::Vector3d(pixel.val[0], pixel.val[1], pixel.val[2]) / 255.;
                            }
                        }
                    }
                }
            } else if (fmt == Ubitrack::Vision::Image::LUMINANCE) {

                auto &points = cloud->points_;
                auto &colors = cloud->colors_;
                const auto &pointcloud = *vec3_measurement;

                points.resize(num_valid_pixels);
                colors.resize(num_valid_pixels);

                int cnt = 0;
                for (int i = 0; i < m_current_camera01_image->height(); i++) {
                    for (int j = 0; j < m_current_camera01_image->width(); j++) {
                        if (cnt < num_valid_pixels) {
                            uchar pixel = img.at<uchar>(i, j);
                            auto &p = pointcloud.at(cnt);
                            points[cnt] = Eigen::Vector3d(p(0), p(1), p(2));
                            colors[cnt++] = Eigen::Vector3d(pixel, pixel, pixel) / 255.;
                        }
                    }
                }
            } else {
                LOG4CPP_WARN(logger, "unknown image format: " << fmt);
                return false;
            }
        } else {
            LOG4CPP_WARN(logger, "no pointcloud measurement received");
            return false;
        }
    } catch (std::exception &e) {
        LOG4CPP_WARN(logger, "error retrieving measurement for camera01: " << e.what());
        return false;
    }

    return true;
}

bool UbitrackPointCloudConnector::camera02_get_pose(Ubitrack::Measurement::Timestamp ts, Eigen::Matrix4d &transform) {
    try {
        Ubitrack::Measurement::Pose cam2_pose = m_pullsink_camera02_pose->get(ts);

        if (cam2_pose) {
            Ubitrack::Math::Quaternion rot = cam2_pose->rotation();
            Ubitrack::Math::Vector3d trans = cam2_pose->translation();

            // ubitrack: rw, rx, ry, rz
            // eigen: x, y, z, w
            Eigen::Quaternion<double> rotation(rot.w(), rot.x(), rot.y(), rot.z());

            Eigen::Vector3d position;
            position(0) = trans(0);
            position(1) = trans(1);
            position(2) = trans(2);

            Eigen::Matrix4d t;
            t.setIdentity();
            t.topLeftCorner<3,3>() = rotation.toRotationMatrix();
            t.topRightCorner<3,1>() = position;

            transform = t;

        } else {
            LOG4CPP_WARN(logger, "Error getting camera2 pose");
            return false;
        }

    } catch (std::exception &e) {
        LOG4CPP_WARN(logger, "error retrieving pose for camera02: " << e.what());
        return false;
    }
    return true;
}

bool UbitrackPointCloudConnector::camera02_get_pointcloud(Ubitrack::Measurement::Timestamp ts, std::shared_ptr<open3d::PointCloud>& cloud)
{
    if (!have_camera02()) {
        return false;
    }
    // we need locking here to prevent concurrent access to m_current_camera01_image (when receiving new frame)
    std::unique_lock<std::mutex> ul( m_textureAccessMutex );

    try {
        Ubitrack::Measurement::PositionList vec3_measurement = m_pullsink_camera02_pointcloud->get(ts);
        Ubitrack::Measurement::ImageMeasurement img_measurement = m_pullsink_camera02_image->get(ts);

        if ((vec3_measurement) && (!vec3_measurement->empty()) && (img_measurement)) {

            size_t num_valid_pixels = vec3_measurement->size();

            auto img = img_measurement->Mat();

            Ubitrack::Vision::Image::PixelFormat fmt = img_measurement->pixelFormat();
            if ((fmt == Ubitrack::Vision::Image::BGRA) || (fmt == Ubitrack::Vision::Image::RGBA)) {

                auto& points = cloud->points_;
                auto& colors = cloud->colors_;
                const auto& pointcloud = *vec3_measurement;

                points.resize(num_valid_pixels);
                colors.resize(num_valid_pixels);

                int cnt = 0;
                for (int i = 0; i < img_measurement->height(); i++) {
                    for (int j = 0; j < img_measurement->width(); j++) {
                        if (cnt < num_valid_pixels) {
                            cv::Vec4b pixel = img.at<cv::Vec4b>(i, j);
                            auto& p = pointcloud.at(cnt);
                            points[cnt] = Eigen::Vector3d(p(0), p(1), p(2));
                            if (fmt == Ubitrack::Vision::Image::BGRA) {
                                colors[cnt++] = Eigen::Vector3d(pixel.val[2], pixel.val[1], pixel.val[0]) / 255.;
                            } else {
                                colors[cnt++] = Eigen::Vector3d(pixel.val[0], pixel.val[1], pixel.val[2]) / 255.;
                            }
                        }
                    }
                }
            } else  if (fmt == Ubitrack::Vision::Image::LUMINANCE) {

                auto& points = cloud->points_;
                auto& colors = cloud->colors_;
                const auto& pointcloud = *vec3_measurement;

                points.resize(num_valid_pixels);
                colors.resize(num_valid_pixels);

                int cnt = 0;
                for (int i = 0; i < m_current_camera01_image->height(); i++) {
                    for (int j = 0; j < m_current_camera01_image->width(); j++) {
                        if (cnt < num_valid_pixels) {
                            uchar pixel = img.at<uchar>(i, j);
                            auto& p = pointcloud.at(cnt);
                            points[cnt] = Eigen::Vector3d(p(0), p(1), p(2));
                            colors[cnt++] = Eigen::Vector3d(pixel, pixel, pixel) / 255.;
                        }
                    }
                }
            } else {
                LOG4CPP_WARN( logger, "unknown image format: " << fmt);
                return false;
            }
        } else {
            LOG4CPP_WARN( logger, "no pointcloud measurement received");
            return false;
        }
    } catch (std::exception &e) {
        LOG4CPP_WARN(logger, "error retrieving measurement for camera02: " << e.what());
        return false;
    }

//    Eigen::Matrix4d transform;
//    if (camera02_get_pose(ts, transform)) {
//        cloud->Transform(transform);
//    }

    return true;
}

bool UbitrackPointCloudConnector::camera03_get_pose(Ubitrack::Measurement::Timestamp ts, Eigen::Matrix4d &transform) {
    try {
        Ubitrack::Measurement::Pose cam3_pose = m_pullsink_camera03_pose->get(ts);

        if (cam3_pose) {
            Ubitrack::Math::Quaternion rot = cam3_pose->rotation();
            Ubitrack::Math::Vector3d trans = cam3_pose->translation();

            // ubitrack: rw, rx, ry, rz
            // eigen: x, y, z, w
            Eigen::Quaternion<double> rotation(rot.w(), rot.x(), rot.y(), rot.z());

            Eigen::Vector3d position;
            position(0) = trans(0);
            position(1) = trans(1);
            position(2) = trans(2);

            Eigen::Matrix4d t;
            t.setIdentity();
            t.topLeftCorner<3,3>() = rotation.toRotationMatrix();
            t.topRightCorner<3,1>() = position;

            transform = t;

        } else {
            LOG4CPP_WARN(logger, "Error getting camera3 pose");
            return false;
        }

    } catch (std::exception &e) {
        LOG4CPP_WARN(logger, "error retrieving pose for camera03: " << e.what());
        return false;
    }
    return true;
}

bool UbitrackPointCloudConnector::camera03_get_pointcloud(Ubitrack::Measurement::Timestamp ts, std::shared_ptr<open3d::PointCloud>& cloud)
{
    if (!have_camera03()) {
        return false;
    }
    // we need locking here to prevent concurrent access to m_current_camera01_image (when receiving new frame)
    std::unique_lock<std::mutex> ul( m_textureAccessMutex );

    try {
        Ubitrack::Measurement::PositionList vec3_measurement = m_pullsink_camera03_pointcloud->get(ts);
        Ubitrack::Measurement::ImageMeasurement img_measurement = m_pullsink_camera03_image->get(ts);

        if ((vec3_measurement) && (!vec3_measurement->empty()) && (img_measurement)) {

            size_t num_valid_pixels = vec3_measurement->size();

            auto img = img_measurement->Mat();

            Ubitrack::Vision::Image::PixelFormat fmt = img_measurement->pixelFormat();
            if ((fmt == Ubitrack::Vision::Image::BGRA) || (fmt == Ubitrack::Vision::Image::RGBA)) {

                auto& points = cloud->points_;
                auto& colors = cloud->colors_;
                const auto& pointcloud = *vec3_measurement;

                points.resize(num_valid_pixels);
                colors.resize(num_valid_pixels);

                int cnt = 0;
                for (int i = 0; i < img_measurement->height(); i++) {
                    for (int j = 0; j < img_measurement->width(); j++) {
                        if (cnt < num_valid_pixels) {
                            cv::Vec4b pixel = img.at<cv::Vec4b>(i, j);
                            auto& p = pointcloud.at(cnt);
                            points[cnt] = Eigen::Vector3d(p(0), p(1), p(2));
                            if (fmt == Ubitrack::Vision::Image::BGRA) {
                                colors[cnt++] = Eigen::Vector3d(pixel.val[2], pixel.val[1], pixel.val[0]) / 255.;
                            } else {
                                colors[cnt++] = Eigen::Vector3d(pixel.val[0], pixel.val[1], pixel.val[2]) / 255.;
                            }
                        }
                    }
                }
            } else  if (fmt == Ubitrack::Vision::Image::LUMINANCE) {

                auto& points = cloud->points_;
                auto& colors = cloud->colors_;
                const auto& pointcloud = *vec3_measurement;

                points.resize(num_valid_pixels);
                colors.resize(num_valid_pixels);

                int cnt = 0;
                for (int i = 0; i < m_current_camera01_image->height(); i++) {
                    for (int j = 0; j < m_current_camera01_image->width(); j++) {
                        if (cnt < num_valid_pixels) {
                            uchar pixel = img.at<uchar>(i, j);
                            auto& p = pointcloud.at(cnt);
                            points[cnt] = Eigen::Vector3d(p(0), p(1), p(2));
                            colors[cnt++] = Eigen::Vector3d(pixel, pixel, pixel) / 255.;
                        }
                    }
                }
            } else {
                LOG4CPP_WARN( logger, "unknown image format: " << fmt);
                return false;
            }
        } else {
            LOG4CPP_WARN( logger, "no pointcloud measurement received");
            return false;
        }
    } catch (std::exception &e) {
        LOG4CPP_WARN(logger, "error retrieving measurement for camera03: " << e.what());
        return false;
    }

//    Eigen::Matrix4d transform;
//    if (camera03_get_pose(ts, transform)) {
//        cloud->Transform(transform);
//    }

    return true;
}


void UbitrackPointCloudConnector::receive_camera01_image(const Ubitrack::Measurement::ImageMeasurement& image)
{
    {
        std::unique_lock<std::mutex> ul(m_textureAccessMutex);
        m_current_camera01_image = image;
    }
    // notify renderer that new frame is available
    set_new_frame(image.time());
}

