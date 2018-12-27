//
// Created by Ulrich Eck on 15.03.18.
//


#include "artekmed/UbitrackPointCloudVisualizer.h"
#include "artekmed/UbitrackViewControl.h"

#include <log4cpp/Category.hh>
static log4cpp::Category& logger(log4cpp::Category::getInstance("ArtekmedP1.UbitrackPointCloudVisualizer"));

namespace open3d {

UbitrackPointCloudVisualizer::UbitrackPointCloudVisualizer()
{
}

UbitrackPointCloudVisualizer::~UbitrackPointCloudVisualizer()
{
}

void UbitrackPointCloudVisualizer::setPointCloud(std::shared_ptr<open3d::PointCloud>& point_cloud) {
    ubitrack_camera01_pointcloud_ptr = point_cloud;
    AddGeometry(point_cloud);
}

bool UbitrackPointCloudVisualizer::StartUbitrack() {
    if (ubitrack_connector_ptr) {
        LOG4CPP_INFO(logger, "Starting Ubitrack Dataflow")
        ubitrack_connector_ptr->start();
        // initialize GPU related parts of Ubitrack after OpenGL Window and Context was created
        // must be called after dataflow was started (components notify openclmanager to become active)
        Ubitrack::Facade::initGPU();
    }
    return true;
}

bool UbitrackPointCloudVisualizer::StopUbitrack(){
    if (ubitrack_connector_ptr)
    LOG4CPP_INFO(logger, "Stopping Ubitrack Dataflow")
        ubitrack_connector_ptr->stop();
    return true;
}


void UbitrackPointCloudVisualizer::SetUbitrackConnector(std::shared_ptr<UbitrackPointCloudConnector>& connector_) throw()
{
//    if (!is_initialized_)
//        throw std::runtime_error("Visualizer must be initialized before setting up the connector");

    ubitrack_connector_ptr = connector_;

    RegisterAnimationCallback(
        [=](open3d::Visualizer *vis) {
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

            Ubitrack::Measurement::Timestamp ts;
            if (!connector->wait_for_frame_timeout(1000, ts)) {
                bool needs_update = false;

                // transfer camera_left_image to opengl texture
                if (ubitrack_camera01_pointcloud_ptr) {
                    if (connector->camera01_get_pointcloud(ts, ubitrack_camera01_pointcloud_ptr)) {
                        view_control->FitInGeometry(*ubitrack_camera01_pointcloud_ptr);
                        needs_update = true;
                    } else {
                        LOG4CPP_WARN(logger, "error retrieving camera image.");
                    }
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


} // open3d