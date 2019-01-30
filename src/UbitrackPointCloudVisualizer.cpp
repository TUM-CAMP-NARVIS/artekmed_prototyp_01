//
// Created by Ulrich Eck on 15.03.18.
//

#include <Core/Core.h>
#include <IO/IO.h>
#include <artekmed/Visualization/Visualization.h>

#include "artekmed/UbitrackPointCloudVisualizer.h"
#include "artekmed/Visualization/Visualizer/UbitrackViewControl.h"

#include <log4cpp/Category.hh>
static log4cpp::Category& logger(log4cpp::Category::getInstance("ArtekmedP1.UbitrackPointCloudVisualizer"));

namespace artekmed {

UbitrackPointCloudVisualizer::UbitrackPointCloudVisualizer()
{
    pointcloud_processor_ptr = std::make_shared<compute::OCLTestProcessor>("HelloWorld");
}

UbitrackPointCloudVisualizer::~UbitrackPointCloudVisualizer()
{
    pointcloud_processor_ptr.reset();
}

void UbitrackPointCloudVisualizer::addPointCloud(std::shared_ptr<open3d::PointCloud>& point_cloud) {
    ubitrack_pointclouds_ptr.emplace_back(point_cloud);
    AddGeometry(point_cloud);
}

bool UbitrackPointCloudVisualizer::StartUbitrack() {
    UbitrackVisualizer::StartUbitrack();
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
    UbitrackVisualizer::StopUbitrack();
    return true;
}


void UbitrackPointCloudVisualizer::SetUbitrackConnector(std::shared_ptr<UbitrackPointCloudConnector>& connector_) throw()
{
//    if (!is_initialized_)
//        throw std::runtime_error("Visualizer must be initialized before setting up the connector");

    ubitrack_connector_ptr = connector_;

    RegisterAnimationCallback(
        [=](artekmed::Visualizer *vis) {

            // The lambda function captures no references to avoid dangling
            // references
            auto view_control = dynamic_cast<UbitrackViewControl*>(view_control_ptr_.get());
            auto connector = ubitrack_connector_ptr.get();
            bool needs_update = false;

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

            if (UbitrackShouldRun()) {
                if (!connector->isRunning()) {
                    LOG4CPP_INFO(logger, "Start Ubitrack.");
                    StartUbitrack();
                    SetupRenderManager();

                    // add debug coordinate frames for all camera poses
                    Ubitrack::Measurement::Timestamp ts = connector->now();
                    for (auto&& cam : connector->cameras()) {
                        Eigen::Matrix4d transform;
                        if (cam->get_camera_pose(ts, transform)) {
                            auto mesh = open3d::CreateMeshCoordinateFrame(0.3);
                            mesh->Transform(transform);
                            AddGeometry(mesh);
                            needs_update = true;

                        }
                    }
                }
            } else {
                if (connector->isRunning()) {
                    LOG4CPP_INFO(logger, "Stop Ubitrack.");
                    StopUbitrack();
                }
            }

#ifdef ___DISABLED
            // check and init OpenCL if needed ..
            auto clmgr = &Ubitrack::Vision::OpenCLManager::singleton();
            if (clmgr->isEnabled()) {
                if (!clmgr->isActive()) {
                    clmgr->activate();
                }
                if (!clmgr->isInitialized()) {
                    // curren context ??
                    clmgr->initializeOpenGL();
                }
            }

            if (clmgr->isInitialized()) {

            }
#endif

            if (cv::ocl::haveOpenCL()) {
                pointcloud_processor_ptr->run_kernel();
            }



            std::vector<Ubitrack::Measurement::Timestamp> tsv;
            if (!connector->wait_for_frame_timeout(3, tsv)) {

                int i = 0;
                for (auto&& cam : connector->cameras()) {
                    Ubitrack::Measurement::Timestamp ts = tsv[i];
                    if ((ts != 0) && (cam->have_camera())) {

                        if (i < ubitrack_pointclouds_ptr.size()) {
                            auto& pc_ptr = ubitrack_pointclouds_ptr.at(i);
                            if (cam->get_pointcloud(ts, pc_ptr)) {
                                view_control->FitInGeometry(*pc_ptr);
                                needs_update = true;
                            } else {
                                LOG4CPP_WARN(logger, "error retrieving from: " << cam->get_name());
                                return false;
                            }
                        }
                    }
                    i++;
                }

                // things have changed, notify renderer
                UpdateWindowTitle();
                return needs_update;

            } else {
                LOG4CPP_TRACE(logger, "no data was received from connector.");
            }

          UpdateWindowTitle();
          return false;
        });
}


} // open3d