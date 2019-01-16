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
        , m_have_camera01(false)
        , m_have_camera02(false)
        , m_have_camera03(false)
{}

bool UbitrackPointCloudConnector::initialize(const std::string& _utql_filename)
{
    if (!UbitrackBaseConnector::initialize(_utql_filename)){
        return false;
    }

    // create sinks/sources
    try {
        m_pushsink_camera01_image = m_utFacade->componentByName<Ubitrack::Components::ApplicationPushSinkVisionImage>("camera01_image");
        m_pullsink_camera01_pointcloud = m_utFacade->componentByName<Ubitrack::Components::ApplicationPullSinkPositionList>("camera01_pointcloud");
        m_have_camera01 = true;
    } catch (std::exception &e) {
        LOG4CPP_ERROR(logger, e.what());
        m_have_camera01 = false;
    }

    try{
        m_pullsink_camera02_image = m_utFacade->componentByName<Ubitrack::Components::ApplicationPullSinkVisionImage>("camera02_image");
        m_pullsink_camera02_pointcloud = m_utFacade->componentByName<Ubitrack::Components::ApplicationPullSinkPositionList>("camera02_pointcloud");
        m_pullsink_camera02_pose = m_utFacade->componentByName<Ubitrack::Components::ApplicationPullSinkPose>("camera02_pose");
        m_have_camera02 = true;
    } catch (std::exception &e) {
        LOG4CPP_ERROR(logger, e.what());
        m_have_camera02 = false;
    }

    try {
        m_pullsink_camera03_image = m_utFacade->componentByName<Ubitrack::Components::ApplicationPullSinkVisionImage>("camera03_image");
        m_pullsink_camera03_pointcloud = m_utFacade->componentByName<Ubitrack::Components::ApplicationPullSinkPositionList>("camera03_pointcloud");
        m_pullsink_camera03_pose = m_utFacade->componentByName<Ubitrack::Components::ApplicationPullSinkPose>("camera03_pose");
        m_have_camera03 = true;
    } catch (std::exception &e) {
        LOG4CPP_ERROR(logger, e.what());
        m_have_camera03 = false;
    }

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
    m_pullsink_camera01_pointcloud.reset();

    m_pullsink_camera02_image.reset();
    m_pullsink_camera02_pointcloud.reset();
    m_pullsink_camera02_pose.reset();

    m_pullsink_camera03_image.reset();
    m_pullsink_camera03_pointcloud.reset();
    m_pullsink_camera03_pose.reset();

    return UbitrackBaseConnector::teardown();
}

bool UbitrackPointCloudConnector::camera01_get_pointcloud(Ubitrack::Measurement::Timestamp ts, std::shared_ptr<open3d::PointCloud>& cloud)
{
    if (!m_have_camera01) {
        return false;
    }
    // we need locking here to prevent concurrent access to m_current_camera01_image (when receiving new frame)
    std::unique_lock<std::mutex> ul( m_textureAccessMutex );

    try {
        Ubitrack::Measurement::PositionList vec3_measurement = m_pullsink_camera01_pointcloud->get(ts);

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

bool UbitrackPointCloudConnector::camera02_get_pointcloud(Ubitrack::Measurement::Timestamp ts, std::shared_ptr<open3d::PointCloud>& cloud)
{
    if (!m_have_camera02) {
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

    return true;
}

bool UbitrackPointCloudConnector::camera03_get_pointcloud(Ubitrack::Measurement::Timestamp ts, std::shared_ptr<open3d::PointCloud>& cloud)
{
    if (!m_have_camera03) {
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

