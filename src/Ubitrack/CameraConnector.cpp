//
// Created by netlabs on 27.01.19.
//

#include "artekmed/Ubitrack/CameraConnector.h"
#include "artekmed/PointCloudProcessing.h"
#include "artekmed/EigenWrapper.h"

#include <utUtil/TracingProvider.h>

//#define DO_TIMING

#include "artekmed/Compute/OCLPointCloudProcessor.h"


#include <log4cpp/Category.hh>
static log4cpp::Category& logger(log4cpp::Category::getInstance("ArtekmedP1.CameraConnector"));


namespace artekmed {
    using namespace open3d;

    VideoCameraConnector::VideoCameraConnector(const std::string& name)
            : m_camera_basename(name)
            , m_haveNewFrame( false )
            , m_lastTimestamp( 0 )
    {
    }

    bool VideoCameraConnector::initialize(Ubitrack::Facade::AdvancedFacade* facade) {
        // create sinks/sources
        // Camera1
        try {
            std::string portname = m_camera_basename + "_image";
            m_pushsink_camera_image = facade->componentByName<Ubitrack::Components::ApplicationPushSinkVisionImage>(portname.c_str());
        } catch (std::exception &e) {
            LOG4CPP_ERROR(logger, e.what());
            m_pushsink_camera_image.reset();
        }
        try{
            std::string portname = m_camera_basename + "_pose";
            m_pullsink_camera_pose = facade->componentByName<Ubitrack::Components::ApplicationPullSinkPose>(portname.c_str());
        } catch (std::exception &e) {
            LOG4CPP_ERROR(logger, e.what());
            m_pullsink_camera_pose.reset();
        }
        try{
            std::string portname = m_camera_basename + "_image_model";
            m_pullsink_camera_image_model = facade->componentByName<Ubitrack::Components::ApplicationPullSinkCameraIntrinsics>(portname.c_str());
        } catch (std::exception &e) {
            LOG4CPP_ERROR(logger, e.what());
            m_pullsink_camera_image_model.reset();
        }

        // Push handler callback
        if (m_pushsink_camera_image) {
            m_pushsink_camera_image->setCallback(boost::bind( &VideoCameraConnector::receive_image, this, _1 ));
        }
        return true;

    }

    bool VideoCameraConnector::teardown(Ubitrack::Facade::AdvancedFacade* facade) {

        // need to unregister callback here !!!
        if (m_pushsink_camera_image) {
            m_pushsink_camera_image->setCallback(NULL);
        }
        // deallocate sinks to be sure there is no activity before deallocating the facade
        m_pushsink_camera_image.reset();
        m_pullsink_camera_pose.reset();
        m_pullsink_camera_image_model.reset();

    }


    void VideoCameraConnector::set_new_frame(Ubitrack::Measurement::Timestamp ts) {
#ifdef HAVE_USDT
        FOLLY_SDT(artekmed_p1, cameraconnector_set_new_frame, m_camera_basename, ts);
#endif

        std::unique_lock<std::mutex> ul( m_waitMutex );
        m_lastTimestamp = ts;
        m_haveNewFrame = true;
        m_waitCondition.notify_one();
    }

    void VideoCameraConnector::receive_image(const Ubitrack::Measurement::ImageMeasurement& image)
    {
#ifdef HAVE_USDT
        FOLLY_SDT(artekmed_p1, cameraconnector_receive_image, m_camera_basename, image.time());
#endif
        {
            std::unique_lock<std::mutex> ul(m_textureAccessMutex);
            m_current_camera_image = image;
        }
        // notify renderer that new frame is available
        set_new_frame(image.time());
    }

    unsigned int VideoCameraConnector::wait_for_frame_timeout(unsigned int timeout_ms, Ubitrack::Measurement::Timestamp& ts) {
        auto timeout = std::chrono::milliseconds(timeout_ms);

        std::unique_lock<std::mutex> ul( m_waitMutex );
        if(m_waitCondition.wait_for( ul , timeout) == std::cv_status::no_timeout) {
            if (m_haveNewFrame) {
                ts = m_lastTimestamp;
                m_haveNewFrame = false;
#ifdef HAVE_USDT
                FOLLY_SDT(artekmed_p1, cameraconnector_wait_for_frame_timeout, m_camera_basename, ts);
#endif
                return 0;
            } else {
                // no new frame
                ts = 0;
                return 1;
            }
        } else {
            // timeout
            ts = 0;
            return 2;
        }
    }

    bool VideoCameraConnector::get_camera_pose(Ubitrack::Measurement::Timestamp ts, Eigen::Matrix4d &transform) {

        if (!m_pullsink_camera_pose) {
            return false;
        }

        try {
            Ubitrack::Measurement::Pose cam_pose = m_pullsink_camera_pose->get(ts);

            if (cam_pose) {
                Eigen::Quaternion<double> rotation = Ubitrack::Math::Wrapper::EigenQuaternionCast().to_eigen(cam_pose->rotation());
                Eigen::Vector3d position = Ubitrack::Math::Wrapper::EigenVectorCast().to_eigen<double, 3>(cam_pose->translation());

                Eigen::Matrix4d t;
                t.setIdentity();
                t.topLeftCorner<3,3>() = rotation.toRotationMatrix();
                t.topRightCorner<3,1>() = position;

                transform = t;

            } else {
                LOG4CPP_WARN(logger, "Error getting pose for " << m_camera_basename);
                return false;
            }

        } catch (std::exception &e) {
            LOG4CPP_ERROR(logger, "error retrieving pose for " << m_camera_basename << ": " << e.what());
            return false;
        }
        return true;
    }

    bool VideoCameraConnector::get_image(std::shared_ptr<UbitrackImage>& image) {

        try {

            image->ubitrack_image_ptr = m_current_camera_image;

        } catch (std::exception &e) {
            LOG4CPP_ERROR(logger, "error retrieving current image for " << m_camera_basename << ": " << e.what());
            return false;
        }
        return true;

    }

    bool VideoCameraConnector::get_image_intrinsics(Ubitrack::Measurement::Timestamp ts, Eigen::Matrix3d& intrinsics) {
        if (!m_pullsink_camera_image_model) {
            return false;
        }

        try {
            Ubitrack::Measurement::CameraIntrinsics intr = m_pullsink_camera_image_model->get(ts);

            if (intr) {
                intrinsics = Ubitrack::Math::Wrapper::EigenIntrinsicMatrixCast().to_eigen(intr->matrix);

            } else {
                LOG4CPP_WARN(logger, "Error getting image model for " << m_camera_basename);
                return false;
            }

        } catch (std::exception &e) {
            LOG4CPP_ERROR(logger, "error retrieving image model for " << m_camera_basename << ": " << e.what());
            return false;
        }
        return true;

    }


    bool RGBDCameraConnector::initialize(Ubitrack::Facade::AdvancedFacade* facade) {
        VideoCameraConnector::initialize(facade);

        try{
            std::string portname = m_camera_basename + "_depth";
            m_pullsink_camera_depth = facade->componentByName<Ubitrack::Components::ApplicationPullSinkVisionImage>(portname.c_str());
        } catch (std::exception &e) {
            LOG4CPP_ERROR(logger, e.what());
            m_pullsink_camera_depth.reset();
        }
        try {
            std::string portname = m_camera_basename + "_pointcloud";
            m_pullsink_camera_pointcloud = facade->componentByName<Ubitrack::Components::ApplicationPullSinkPositionList>(portname.c_str());
        } catch (std::exception &e) {
            LOG4CPP_ERROR(logger, e.what());
            m_pullsink_camera_pointcloud.reset();
        }
        try{
            std::string portname = m_camera_basename + "_depth_model";
            m_pullsink_camera_depth_model = facade->componentByName<Ubitrack::Components::ApplicationPullSinkCameraIntrinsics>(portname.c_str());
        } catch (std::exception &e) {
            LOG4CPP_ERROR(logger, e.what());
            m_pullsink_camera_depth_model.reset();
        }
        try{
            std::string portname = m_camera_basename + "_depth2color";
            m_pullsink_depth2color_pose = facade->componentByName<Ubitrack::Components::ApplicationPullSinkPose>(portname.c_str());
        } catch (std::exception &e) {
            LOG4CPP_ERROR(logger, e.what());
            m_pullsink_depth2color_pose.reset();
        }

        return true;
    }

    bool RGBDCameraConnector::teardown(Ubitrack::Facade::AdvancedFacade* facade) {

        VideoCameraConnector::teardown(facade);

        m_pullsink_camera_depth.reset();
        m_pullsink_camera_pointcloud.reset();
        m_pullsink_camera_depth_model.reset();
        m_pullsink_depth2color_pose.reset();

        m_depthregistration.reset();

        return true;
    }

    bool RGBDCameraConnector::get_depth2color_pose(Ubitrack::Measurement::Timestamp ts, Eigen::Matrix4d &transform) {
        if (!m_pullsink_depth2color_pose) {
            return false;
        }

        try {
            Ubitrack::Measurement::Pose pose = m_pullsink_depth2color_pose->get(ts);

            if (pose) {
                Eigen::Quaternion<double> rotation = Ubitrack::Math::Wrapper::EigenQuaternionCast().to_eigen(pose->rotation());
                Eigen::Vector3d position = Ubitrack::Math::Wrapper::EigenVectorCast().to_eigen<double, 3>(pose->translation());

                Eigen::Matrix4d t;
                t.setIdentity();
                t.topLeftCorner<3,3>() = rotation.toRotationMatrix();
                t.topRightCorner<3,1>() = position;

                transform = t;

            } else {
                LOG4CPP_WARN(logger, "Error getting depth2color for " << m_camera_basename);
                return false;
            }

        } catch (std::exception &e) {
            LOG4CPP_ERROR(logger, "error retrieving depth2color for " << m_camera_basename << ": " << e.what());
            return false;
        }
        return true;
    }

    bool RGBDCameraConnector::get_depth_intrinsics(Ubitrack::Measurement::Timestamp ts, Eigen::Matrix3d& intrinsics) {
        if (!m_pullsink_camera_depth_model) {
            return false;
        }

        try {
            Ubitrack::Measurement::CameraIntrinsics intr = m_pullsink_camera_depth_model->get(ts);

            if (intr) {
                intrinsics = Ubitrack::Math::Wrapper::EigenIntrinsicMatrixCast().to_eigen(intr->matrix);

            } else {
                LOG4CPP_WARN(logger, "Error getting depth model for " << m_camera_basename);
                return false;
            }

        } catch (std::exception &e) {
            LOG4CPP_ERROR(logger, "error retrieving depth model for " << m_camera_basename << ": " << e.what());
            return false;
        }
        return true;

    }



    bool RGBDCameraConnector::get_pointcloud(Ubitrack::Measurement::Timestamp ts, std::shared_ptr<open3d::PointCloud>& cloud) {
        // if sinks are not complete - stop
        if (!have_camera()) {
            return false;
        }

        // if no timestamp is provided - stop
        if (ts == 0) {
            return false;
        }

#ifdef HAVE_USDT
        FOLLY_SDT(artekmed_p1, cameraconnector_get_pointcloud_begin, m_camera_basename, ts);
#endif

//        LOG4CPP_INFO(logger, "get_pointcloud (" << m_camera_basename << ", " << ts << ")");

        bool have_camera_pose = false;
        bool have_depth_model = false;
        bool have_depth2color = false;

        Ubitrack::Measurement::ImageMeasurement color_image;
        Ubitrack::Measurement::ImageMeasurement depth_image;
        Ubitrack::Measurement::CameraIntrinsics image_model;

        Ubitrack::Measurement::CameraIntrinsics depth_model;
        Ubitrack::Measurement::Pose camera_pose;
        Ubitrack::Measurement::Pose camera_depth2color;

        {
            // we need locking here to prevent concurrent access to m_current_camera01_image (when receiving new frame)
            std::unique_lock<std::mutex> ul(m_textureAccessMutex);



            // should be configurable ..
            float depth_scale_factor = 0.001;

            try {
                color_image = m_current_camera_image;
                depth_image = m_pullsink_camera_depth->get(ts);
                image_model = m_pullsink_camera_image_model->get(ts);

            } catch (std::exception &e) {
                LOG4CPP_ERROR(logger, "error retrieving measurement for " << m_camera_basename << ": " << e.what());
                return false;
            }


            try {
                if (m_pullsink_camera_depth_model) {
                    depth_model = m_pullsink_camera_depth_model->get(ts);
                    have_depth_model = true;
                }
            } catch (std::exception &e) {
                depth_model = image_model;
            }

            try {
                if (m_pullsink_camera_pose) {
                    camera_pose = m_pullsink_camera_pose->get(ts);
                    have_camera_pose = true;
                }
            } catch (std::exception &e) {
                camera_pose = Ubitrack::Measurement::Pose(ts, Ubitrack::Math::Pose());
            }

            try {
                if (m_pullsink_depth2color_pose) {
                    camera_depth2color = m_pullsink_depth2color_pose->get(ts);
                    have_depth2color = true;
                }
            } catch (std::exception &e) {
                camera_depth2color = Ubitrack::Measurement::Pose(ts, Ubitrack::Math::Pose());
            }

            auto depth_img = depth_image->Mat();
            auto color_img = color_image->Mat();

            Eigen::Matrix3d intrinsics_color = Ubitrack::Math::Wrapper::EigenIntrinsicMatrixCast().to_eigen(
                    image_model->matrix);

            if ((have_depth_model) && (have_depth2color)) {

#ifdef HAVE_USDT
                FOLLY_SDT(artekmed_p1, cameraconnector_get_pointcloud_convert1, m_camera_basename, ts);
#endif

                // this currently suports only depthmaps from intel realsense as uint16
                if ((depth_image->pixelFormat() != Ubitrack::Vision::Image::DEPTH) || (depth_image->bitsPerPixel() != 16)) {
                    LOG4CPP_ERROR(logger, "unsupported depth format - need DEPTH / UINT16!");
                    return false;
                }

                // for now works only with RS D435 depth images 24bit BGR
                if ((color_image->pixelFormat() != Ubitrack::Vision::Image::BGR) || (color_image->bitsPerPixel() != 24)) {
                    LOG4CPP_ERROR(logger, "unsupported color format - need BGR / CV_8UC3");
                    return false;
                }

                Eigen::Matrix3d intrinsics_depth = Ubitrack::Math::Wrapper::EigenIntrinsicMatrixCast().to_eigen(
                        depth_model->matrix);

                Eigen::Matrix4d depth2color_tf;
                {
                    Eigen::Quaternion<double> rotation = Ubitrack::Math::Wrapper::EigenQuaternionCast().to_eigen(
                            camera_depth2color->rotation());
                    Eigen::Vector3d position = Ubitrack::Math::Wrapper::EigenVectorCast().to_eigen<double, 3>(
                            camera_depth2color->translation());

                    depth2color_tf.setIdentity();
                    depth2color_tf.topLeftCorner<3, 3>() = rotation.toRotationMatrix();
                    depth2color_tf.topRightCorner<3, 1>() = position;
                }

                // compute pointcloud and colors
                buildColoredPointCloud(depth_img, color_img,
                                       intrinsics_depth.cast<float>(), intrinsics_color.cast<float>(), depth2color_tf.cast<float>(),
                                       *cloud, depth_scale_factor);


            } else {

#ifdef HAVE_USDT
                FOLLY_SDT(artekmed_p1, cameraconnector_get_pointcloud_convert2, m_camera_basename, ts);
#endif

                // this is a RGBD Camera without offset between Color and Depth (e.g. Stereolabs ZED)

                auto num_pixels_color = color_image->width() * color_image->height();
                auto num_pixels_depth = depth_image->width() * depth_image->height();

                if (num_pixels_color != num_pixels_depth) {
                    LOG4CPP_ERROR(logger, "Color and Depth image for ZED Camera must have the same size!");
                    return false;
                }

                // for now works only with ZED depth images 32bit float
                if ((depth_image->pixelFormat() != Ubitrack::Vision::Image::DEPTH) || (depth_image->bitsPerPixel() != 32)) {
                    LOG4CPP_ERROR(logger, "unsupported depth format - need DEPTH / 32F!");
                    return false;
                }

                // for now works only with ZED depth images 32bit BGRA
                if ((color_image->pixelFormat() != Ubitrack::Vision::Image::BGRA) || (color_image->bitsPerPixel() != 32)) {
                    LOG4CPP_ERROR(logger, "unsupported color format - need BGRA / CV_8UC4");
                    return false;
                }

//                if (cv::ocl::haveOpenCL()) {
//
//                    if (!m_depthregistration) {
//                        LOG4CPP_INFO(logger, "Initialize Depth Processing");
//                        m_depthregistration.reset(DepthRegistration::New());
//
//                        cv::Mat depth_intr(cv::Size(3, 3), CV_32F);
//                        for (int i = 0; i < 3; i++)
//                            for (int j = 0; j < 3; j++)
//                                depth_intr.at<float>(i,j) = intrinsics_color(i,j);
//
//                        cv::Size depth_size(depth_image->width(), depth_image->height());
//                        m_depthregistration->init(depth_intr, depth_size, 0.001);
//                    }
//
//                    if (m_depthregistration) {
//                        cv::Mat points;
//                        LOG4CPP_INFO(logger, "call depthToPoints");
//                        m_depthregistration->depthToPoints(depth_img, points);
//
//                        size_t num_points = points.total();
//                        cloud->points_.resize(num_points);
//                        cloud->colors_.resize(num_points);
//
//                        for (int i = 0; i < num_points; i++ ) {
//                            const int x = i % points.cols;
//                            const int y = i / points.cols;
//
//                            auto &pt = cloud->points_[i];
//                            cv::Vec3f pos = points.at<cv::Vec3f>(y, x);
//                            pt(0) = pos(0);
//                            pt(1) = pos(1);
//                            pt(2) = pos(2);
//
//                            cv::Vec4b pixel = color_img.at<cv::Vec4b>(y, x);
//                            cloud->colors_[i] = Eigen::Vector3d(pixel.val[2], pixel.val[1], pixel.val[0]) / 255.;
//
//                        }
//                    }
//
//                } else
                {
                    buildColoredPointCloud(depth_img, color_img, intrinsics_color.cast<float>(), *cloud, depth_scale_factor);
                }

            }
        }

        if (have_camera_pose) {
#ifdef HAVE_USDT
            FOLLY_SDT(artekmed_p1, cameraconnector_get_pointcloud_transform, m_camera_basename, ts);
#endif
            Eigen::Matrix4d origin_tf;
            {
                Eigen::Quaternion<double> rotation = Ubitrack::Math::Wrapper::EigenQuaternionCast().to_eigen(camera_pose->rotation());
                Eigen::Vector3d position = Ubitrack::Math::Wrapper::EigenVectorCast().to_eigen<double, 3>(camera_pose->translation());

                origin_tf.setIdentity();
                origin_tf.topLeftCorner<3,3>() = rotation.toRotationMatrix();
                origin_tf.topRightCorner<3,1>() = position;
            }

            // transform into origin coordinates
            cloud->Transform(origin_tf);

        }

#ifdef HAVE_USDT
        FOLLY_SDT(artekmed_p1, cameraconnector_get_pointcloud_end, m_camera_basename, ts);
#endif

        return true;
    }
}