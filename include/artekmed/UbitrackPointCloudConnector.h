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

private:
    boost::shared_ptr<Ubitrack::Components::ApplicationPushSinkVisionImage>    m_pushsink_camera01_image;
    boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkPositionList>   m_pullsink_camera01_pointcloud;

    boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkVisionImage>    m_pullsink_camera02_image;
    boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkPositionList>   m_pullsink_camera02_pointcloud;
    boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkPose>           m_pullsink_camera02_pose;

    boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkVisionImage>    m_pullsink_camera03_image;
    boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkPositionList>   m_pullsink_camera03_pointcloud;
    boost::shared_ptr<Ubitrack::Components::ApplicationPullSinkPose>           m_pullsink_camera03_pose;

    std::mutex m_textureAccessMutex;
    Ubitrack::Measurement::ImageMeasurement m_current_camera01_image;

};

#endif //ARTEKMED_UBITRACKSINGLECAMERACONNECTOR_H
