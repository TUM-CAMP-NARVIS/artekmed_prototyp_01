//
// Created by Ulrich Eck on 15.03.18.
//

#ifndef BASIC_FACADE_DEMO_UBITRACKSINGLECAMERACONNECTOR_H
#define BASIC_FACADE_DEMO_UBITRACKSINGLECAMERACONNECTOR_H

#include <Eigen/Core>
#include "basic_facade_demo/UbitrackBaseConnector.h"

class UbitrackSingleCameraConnector: public UbitrackBaseConnector {
public:
    explicit UbitrackSingleCameraConnector(const std::string& _components_path);
    virtual ~UbitrackSingleCameraConnector() = default;

    // public api

    // first camera input
    // naming already reflects future extensions to stereo camera setup
    bool camera_left_get_model(TimestampT ts, double near, double far,
            Eigen::Matrix4d& projection_matrix, Eigen::Matrix3d& intrinsics_matrix, Eigen::Vector2i& resolution);
    bool camera_left_get_pose(TimestampT ts, Eigen::Matrix4d& pose);
    bool camera_left_get_current_image(std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement >& img);


    // private api (still needs to be public unless we declare friend classes, which is considered to be bad practice.)
    // extend methods to connnect/disconnect sinks
    bool initialize(const std::string& _utql_filename) override;
    bool teardown() override;

    // handlers for push sinks
    void receive_camera_left_image(std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement>& pose);

private:
    std::unique_ptr<Ubitrack::Facade::BasicPushSink< Ubitrack::Facade::BasicImageMeasurement >>            m_pushsink_camera_image_left;
    std::unique_ptr<Ubitrack::Facade::BasicPullSink< Ubitrack::Facade::BasicCameraIntrinsicsMeasurement >> m_pullsink_camera_model_left;
    std::unique_ptr<Ubitrack::Facade::BasicPullSink< Ubitrack::Facade::BasicPoseMeasurement >>             m_pullsink_camera_pose_left;

    std::mutex m_textureAccessMutex;
    std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement> m_current_camera_left_image;

};





#endif //BASIC_FACADE_DEMO_UBITRACKSINGLECAMERACONNECTOR_H
