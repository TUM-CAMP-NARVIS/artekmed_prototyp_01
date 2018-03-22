//
// Created by Ulrich Eck on 15.03.18.
//


#include "basic_facade_demo/UbitrackSingleCameraVisualizer.h"
#include "basic_facade_demo/UbitrackViewControl.h"

#include <log4cpp/Category.hh>
static log4cpp::Category& logger(log4cpp::Category::getInstance("BasicFacadeExample.UbitrackSingleCameraVisualizer"));

namespace three {

UbitrackSingleCameraVisualizer::UbitrackSingleCameraVisualizer()
{
}

UbitrackSingleCameraVisualizer::~UbitrackSingleCameraVisualizer()
{
}

void UbitrackSingleCameraVisualizer::setCameraImage(std::shared_ptr<three::UbitrackImage>& camera_image) {
    ubitrack_camera_image_ptr = camera_image;
    AddUbitrackImage(camera_image);
}

bool UbitrackSingleCameraVisualizer::StartUbitrack() {
    if (ubitrack_connector_ptr) {
        LOG4CPP_INFO(logger, "Starting Ubitrack Dataflow")
        ubitrack_connector_ptr->start();
        // initialize GPU related parts of Ubitrack after OpenGL Window and Context was created
        // must be called after dataflow was started (components notify openclmanager to become active)
        Ubitrack::Facade::initGPU();
    }
    return true;
}

bool UbitrackSingleCameraVisualizer::StopUbitrack(){
    if (ubitrack_connector_ptr)
    LOG4CPP_INFO(logger, "Stopping Ubitrack Dataflow")
        ubitrack_connector_ptr->stop();
    return true;
}


void UbitrackSingleCameraVisualizer::SetUbitrackConnector(std::shared_ptr<UbitrackSingleCameraConnector>& connector_) throw()
{
//    if (!is_initialized_)
//        throw std::runtime_error("Visualizer must be initialized before setting up the connector");

    ubitrack_connector_ptr = connector_;

    RegisterAnimationCallback(
        [=](three::Visualizer *vis) {
            // The lambda function captures no references to avoid dangling
            // references
            auto view_control = dynamic_cast<UbitrackViewControl*>(view_control_ptr_.get());
            auto connector = ubitrack_connector_ptr.get();

            if (!connector) {
                LOG4CPP_ERROR(logger, "connector not available - removing connector callback.");
                RegisterAnimationCallback(nullptr);
                return false;
            }

            // unregister callback if no dataflow is loaded
            if (!connector->isLoaded()) {
                LOG4CPP_ERROR(logger, "no dataflow loaded - removing connector callback.");
                RegisterAnimationCallback(nullptr);
                return false;
            }

            if (!connector->isRunning()) {
              StartUbitrack();
            }

            TimestampT ts;
            if (!connector->wait_for_frame_timeout(1000, ts)) {
                bool needs_update = false;

                // retrieve camera left intrinsics information
                Eigen::Matrix3d intrinsics_left;
                Eigen::Matrix4d projection_left;
                Eigen::Vector2i resolution_left;
                if(connector->camera_left_get_model(ts, view_control->GetNear(), view_control->GetFar(),
                        projection_left, intrinsics_left, resolution_left)) {
                    view_control->SetCameraModel(projection_left, intrinsics_left, resolution_left);
                    needs_update = true;
                } else {
                    LOG4CPP_WARN(logger, "error retrieving intrinsics.");
                }

                // transfer camera_left_image to opengl texture
                if (connector->camera_left_get_current_image(ubitrack_camera_image_ptr->ubitrack_image_ptr)) {
                    needs_update = true;
                } else {
                    LOG4CPP_WARN(logger, "error retrieving camera image.");
                }

                Eigen::Matrix4d extrinsics_left;
                if(connector->camera_left_get_pose(ts, extrinsics_left)){
                    view_control->SetCameraExtrinsics(extrinsics_left);
                    needs_update = true;
                } else {
                    LOG4CPP_WARN(logger, "error retrieving extrinsics.");
                }

                // things have changed, notify renderer
                UpdateWindowTitle();
                return needs_update;

            } else {
                LOG4CPP_WARN(logger, "no data was received from connector.");
            }

          UpdateWindowTitle();
          return false;
        });
}


} // three