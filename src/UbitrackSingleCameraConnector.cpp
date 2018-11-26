//
// Created by Ulrich Eck on 15.03.18.
//

// implementation of UbitrackSingleCameraConnector

#include <functional>
#include <Eigen/Geometry>
#include "basic_facade_demo/UbitrackSingleCameraConnector.h"

#include <log4cpp/Category.hh>
static log4cpp::Category& logger(log4cpp::Category::getInstance("BasicFacadeExample.UbitrackSingleCameraConnector"));


UbitrackSingleCameraConnector::UbitrackSingleCameraConnector(const std::string& _components_path)
        : UbitrackBaseConnector(_components_path)
{}

bool UbitrackSingleCameraConnector::initialize(const std::string& _utql_filename)
{
    if (!UbitrackBaseConnector::initialize(_utql_filename)){
        return false;
    }

    // create sinks/sources
    m_pushsink_camera_image_left = std::unique_ptr<Ubitrack::Facade::BasicPushSink< Ubitrack::Facade::BasicImageMeasurement >>(
            m_utFacade->getPushSink<Ubitrack::Facade::BasicImageMeasurement>("camera_image_left")
    );

    m_pullsink_camera_model_left = std::unique_ptr<Ubitrack::Facade::BasicPullSink< Ubitrack::Facade::BasicCameraIntrinsicsMeasurement >>(
            m_utFacade->getPullSink<Ubitrack::Facade::BasicCameraIntrinsicsMeasurement >("camera_intrinsics_left")
    );

    m_pullsink_camera_pose_left = std::unique_ptr<Ubitrack::Facade::BasicPullSink< Ubitrack::Facade::BasicPoseMeasurement >>(
            m_utFacade->getPullSink<Ubitrack::Facade::BasicPoseMeasurement>("camera_pose_left")
    );

    if (m_pushsink_camera_image_left) {
        m_pushsink_camera_image_left->registerCallback(std::bind(&UbitrackSingleCameraConnector::receive_camera_left_image, this, std::placeholders::_1));
    }
    return true;
}

bool UbitrackSingleCameraConnector::teardown()
{
    // need to unregister callback here !!!
    if (m_pushsink_camera_image_left) {
        m_pushsink_camera_image_left->unregisterCallback();
    }
    // deallocate sinks to be sure there is no activity before deallocating the facade
    m_pushsink_camera_image_left.release();
    m_pullsink_camera_model_left.release();
    m_pullsink_camera_pose_left.release();

    return UbitrackBaseConnector::teardown();
}

bool UbitrackSingleCameraConnector::camera_left_get_model(TimestampT ts, double n, double f,
    Eigen::Matrix4d& projection, Eigen::Matrix3d& intrinsics, Eigen::Vector2i& resolution)
{
    if (!m_pullsink_camera_model_left) {
        LOG4CPP_ERROR(logger, "pullsinks are not connected");
        return false;
    }

    try {
        std::shared_ptr<Ubitrack::Facade::BasicCameraIntrinsicsMeasurement > m_model = m_pullsink_camera_model_left->get(ts);
        if (!m_model) {
            LOG4CPP_ERROR(logger, "no measurement for camera resolution");
            return false;
        }
        // retrieve camera resolution
        std::vector<double> v_res(2);
        m_model->getResolution(v_res);
        resolution(0) = (int)(v_res.at(0));
        resolution(1) = (int)(v_res.at(1));

        // retrieve intrinscs matrix
        std::vector<double> v_intr(9);
        m_model->get(v_intr);
        intrinsics = Eigen::Matrix3d(v_intr.data());

        // compute projection matrix assuming l=0, t=0, r=width, b=height
        std::vector<double> v_proj(16);
		double right = resolution(0);
		double top = resolution(1);
        m_model->getOpenGLProjectionMatrix(0., right, 0., top, n, f, v_proj);
        projection = Eigen::Matrix4d(v_proj.data());

    } catch( std::exception &e) {
        LOG4CPP_ERROR(logger, "error pulling intrinsics: " << e.what());
        return false;
    }
    return true;
}

bool UbitrackSingleCameraConnector::camera_left_get_current_image(std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement >& img)
{
    // we need locking here to prevent concurrent access to m_current_camera_left_image (when receiving new frame)
    std::unique_lock<std::mutex> ul( m_textureAccessMutex );
    img = m_current_camera_left_image;
    return true;
}

bool UbitrackSingleCameraConnector::camera_left_get_pose(const TimestampT ts, Eigen::Matrix4d& pose)
{
    if (m_pullsink_camera_pose_left == nullptr) {
        LOG4CPP_ERROR(logger, "pullsink is not connected");
        return false;
    }
    try {
        std::vector<double> v_pose(7);
        std::shared_ptr<Ubitrack::Facade::BasicPoseMeasurement > m_pose = m_pullsink_camera_pose_left->get(ts);
        if (!m_pose) {
            LOG4CPP_ERROR(logger, "no measurement for camera pose");
            return false;
        }
        m_pose->get(v_pose);

        // ubitrack: rw, rx, ry, rz
        // eigen: x, y, z, w
        Eigen::Quaternion<double> rotation(v_pose[4], v_pose[5], v_pose[6], v_pose[3]);

        Eigen::Vector3d position;
        position(0) = v_pose[0];
        position(1) = v_pose[1];
        position(2) = v_pose[2];

        Eigen::Matrix4d transform;
        transform.setIdentity();
        transform.topLeftCorner<3,3>() = rotation.toRotationMatrix();
        transform.topRightCorner<3,1>() = position;

        pose = transform;

    } catch( std::exception &e) {
        LOG4CPP_ERROR(logger, "error pulling camera pose: " << e.what());
        return false;
    } catch(...) {
        LOG4CPP_ERROR(logger, "error pulling camera pose (undefined) ");
        return false;
    }
    return true;
}

void UbitrackSingleCameraConnector::receive_camera_left_image(std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement>& image)
{
    {
        std::unique_lock<std::mutex> ul(m_textureAccessMutex);
        m_current_camera_left_image = image;
    }
    // notify renderer that new frame is available
    set_new_frame(image->time());
}

