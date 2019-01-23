//
// Created by Ulrich Eck on 15.03.18.
//

#ifndef ARTEKMED_UBITRACKSINGLECAMERACONNECTOR_H
#define ARTEKMED_UBITRACKSINGLECAMERACONNECTOR_H

#include <Eigen/Core>
#include <Core/Geometry/PointCloud.h>
#include "artekmed/UbitrackBaseConnector.h"

#include <boost/shared_ptr.hpp>
#include <utVision/Image.h>
#include "utComponents/ApplicationPullSink.h"
#include "utComponents/ApplicationPushSink.h"
#include "utComponents/ApplicationPullSource.h"
#include "utComponents/ApplicationPushSource.h"
#include "utComponents/ApplicationEndpointsVision.h"

class UbitrackPointCloudConnector: public UbitrackBaseConnector {
public:
    explicit UbitrackPointCloudConnector(const std::string& _components_path);
    virtual ~UbitrackPointCloudConnector() = default;

    // public api

    bool camera01_get_pointcloud(Ubitrack::Measurement::Timestamp ts, std::shared_ptr<open3d::PointCloud>& cloud);
    bool camera02_get_pointcloud(Ubitrack::Measurement::Timestamp ts, std::shared_ptr<open3d::PointCloud>& cloud);
    bool camera03_get_pointcloud(Ubitrack::Measurement::Timestamp ts, std::shared_ptr<open3d::PointCloud>& cloud);

    // private api (still needs to be public unless we declare friend classes, which is considered to be bad practice.)
    // extend methods to connnect/disconnect sinks
    bool initialize(const std::string& _utql_filename) override;
    bool teardown() override;

    // handlers for push sinks
    void receive_camera01_image(const Ubitrack::Measurement::ImageMeasurement& img);

    bool have_camera01() {return ((m_pushsink_camera01_image) && (m_pullsink_camera01_depth || m_pullsink_camera01_pointcloud) && (m_pullsink_camera01_pose) && (m_pullsink_camera01_image_model));}
    bool have_camera02() {return ((m_pullsink_camera02_image) && (m_pullsink_camera02_depth || m_pullsink_camera02_pointcloud) && (m_pullsink_camera02_pose) && (m_pullsink_camera02_image_model) && (m_pullsink_camera02_depth_model));}
    bool have_camera03() {return ((m_pullsink_camera03_image) && (m_pullsink_camera03_depth || m_pullsink_camera03_pointcloud) && (m_pullsink_camera03_pose) && (m_pullsink_camera03_image_model) && (m_pullsink_camera03_depth_model));}

private:
    boost::shared_ptr<Ubitrack::Components::ApplicationPushSinkVisionImage>    m_pushsink_camera01_image;
    boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkVisionImage>    m_pullsink_camera01_depth;
    boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkPositionList>   m_pullsink_camera01_pointcloud;
    boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkPose>           m_pullsink_camera01_pose;
    boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkCameraIntrinsics> m_pullsink_camera01_image_model;

    boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkVisionImage>    m_pullsink_camera02_image;
    boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkVisionImage>    m_pullsink_camera02_depth;
    boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkPositionList>   m_pullsink_camera02_pointcloud;
    boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkPose>           m_pullsink_camera02_pose;
    boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkPose>           m_pullsink_camera02_dept2color;
    boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkCameraIntrinsics> m_pullsink_camera02_image_model;
    boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkCameraIntrinsics> m_pullsink_camera02_depth_model;

    boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkVisionImage>    m_pullsink_camera03_image;
    boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkVisionImage>    m_pullsink_camera03_depth;
    boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkPositionList>   m_pullsink_camera03_pointcloud;
    boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkPose>           m_pullsink_camera03_pose;
    boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkPose>           m_pullsink_camera03_dept2color;
    boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkCameraIntrinsics> m_pullsink_camera03_image_model;
    boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkCameraIntrinsics> m_pullsink_camera03_depth_model;

    std::mutex m_textureAccessMutex;
    Ubitrack::Measurement::ImageMeasurement m_current_camera01_image;
};

#endif //ARTEKMED_UBITRACKSINGLECAMERACONNECTOR_H
