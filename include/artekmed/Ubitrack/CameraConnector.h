//
// Created by netlabs on 27.01.19.
//

#ifndef ARTEKMED_P1_CAMERACONNECTOR_H
#define ARTEKMED_P1_CAMERACONNECTOR_H

#include <memory>
#include <vector>
#include <condition_variable>
#include <mutex>
#include <chrono>
#include <tuple>

#include <Eigen/Core>
#include <Core/Geometry/PointCloud.h>

#include <boost/shared_ptr.hpp>
#include <utVision/Image.h>
#include <utFacade/AdvancedFacade.h>
#include "utComponents/ApplicationPullSink.h"
#include "utComponents/ApplicationPushSink.h"
#include "utComponents/ApplicationPullSource.h"
#include "utComponents/ApplicationPushSource.h"
#include "utComponents/ApplicationEndpointsVision.h"
#include "artekmed/Visualization/Utility/UbitrackImage.h"

namespace artekmed {


    class VideoCameraConnector {

    public:

        explicit VideoCameraConnector(const std::string& name);
        virtual ~VideoCameraConnector () = default;

        // lifecycle
        virtual bool initialize(Ubitrack::Facade::AdvancedFacade* facade);
        virtual bool teardown(Ubitrack::Facade::AdvancedFacade* facade);

        // basics
        const std::string& get_name() {return m_camera_basename;}

        // status
        virtual bool have_camera() {return ((m_pushsink_camera_image) && (m_pullsink_camera_image_model));}

        /*
         * waits for an image to be pushed and returns its timestamp
         */
        unsigned int wait_for_frame_timeout(unsigned int timeout_ms, Ubitrack::Measurement::Timestamp& ts);


        // get data from connector
        bool get_image(std::shared_ptr<UbitrackImage>& image);
        bool get_camera_pose(Ubitrack::Measurement::Timestamp ts, Eigen::Matrix4d& pose);
        bool get_image_intrinsics(Ubitrack::Measurement::Timestamp ts, Eigen::Matrix3d& intrinsics);


        // callbacks
        void receive_image(const Ubitrack::Measurement::ImageMeasurement& img);

    protected:
        void set_new_frame(Ubitrack::Measurement::Timestamp ts);

        std::string m_camera_basename;

        boost::shared_ptr<Ubitrack::Components::ApplicationPushSinkVisionImage>      m_pushsink_camera_image;
        boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkPose>             m_pullsink_camera_pose;
        boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkCameraIntrinsics> m_pullsink_camera_image_model;

        bool m_haveNewFrame;
        Ubitrack::Measurement::Timestamp m_lastTimestamp;

        std::mutex m_waitMutex;
        std::condition_variable m_waitCondition;

        std::mutex m_textureAccessMutex;
        Ubitrack::Measurement::ImageMeasurement m_current_camera_image;


    };



    class RGBDCameraConnector : public VideoCameraConnector {

    public:

        explicit RGBDCameraConnector(const std::string& name) : VideoCameraConnector(name) {};
        virtual ~RGBDCameraConnector () = default;


        bool get_pointcloud(Ubitrack::Measurement::Timestamp ts, std::shared_ptr<open3d::PointCloud>& cloud);
        bool get_depth2color_pose(Ubitrack::Measurement::Timestamp ts, Eigen::Matrix4d& pose);
        bool get_depth_intrinsics(Ubitrack::Measurement::Timestamp ts, Eigen::Matrix3d& intrinsics);

        bool initialize(Ubitrack::Facade::AdvancedFacade* facade) override;
        bool teardown(Ubitrack::Facade::AdvancedFacade* facade) override;

        bool have_camera() override
        {
            return ((VideoCameraConnector::have_camera()) && (m_pullsink_camera_depth));
        }

    protected:
        boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkVisionImage>    m_pullsink_camera_depth;
        boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkPositionList>   m_pullsink_camera_pointcloud;
        boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkPose>           m_pullsink_depth2color_pose;
        boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkCameraIntrinsics> m_pullsink_camera_depth_model;

    };

} // artekmed
#endif //ARTEKMED_P1_CAMERACONNECTOR_H
