//
// Created by Ulrich Eck on 15.03.18.
//


#include "basic_facade_demo/UbitrackSingleCameraVisualizer.h"
#include "basic_facade_demo/UbitrackViewControl.h"


namespace three {

UbitrackSingleCameraVisualizer::UbitrackSingleCameraVisualizer()
{
}

UbitrackSingleCameraVisualizer::~UbitrackSingleCameraVisualizer()
{
}

void UbitrackSingleCameraVisualizer::SetUbitrackConnector(std::shared_ptr<UbitrackSingleCameraConnector>& connector_) throw()
{
    if (!is_initialized_)
        throw std::runtime_error("Visualizer must be initialized before setting up the connector");

    ubitrack_connector_ptr = connector_;

    // also reset textureupdate
    ubitrack_textureupdate_left_ptr = std::unique_ptr<Ubitrack::Facade::BasicTextureUpdate>(new Ubitrack::Facade::BasicTextureUpdate());

    RegisterAnimationCallback(
        [=](three::Visualizer *vis) {
            // The lambda function captures no references to avoid dangling
            // references
            auto &view_control = (UbitrackViewControl &)(view_control_ptr_);
            auto &textureupdate = (Ubitrack::Facade::BasicTextureUpdate &)(ubitrack_textureupdate_left_ptr);
            auto connector = ubitrack_connector_ptr.get();

            if (!connector) {
                PrintWarning("[UbitrackSingleCameraVisualizer] connector not available - removing connector callback.\n");
                RegisterAnimationCallback(nullptr);
                return false;
            }

            // unregister callback if no dataflow is loaded
            if (!connector->isLoaded()) {
                PrintWarning("[UbitrackSingleCameraVisualizer] no dataflow loaded - removing connector callback.\n");
                RegisterAnimationCallback(nullptr);
                return false;
            }

            if (!connector->isRunning()) {
              return false;
            }

            TimestampT ts;
            if (!connector->wait_for_frame_timeout(100, ts)) {

                // retrieve camera left intrinsics information
                Eigen::Matrix3d intrinsics_left;
                Eigen::Vector2i resolution_left;
                if(connector->camera_left_get_intrinsics(ts, intrinsics_left, resolution_left)) {
                    view_control.SetCameraIntrinsics(intrinsics_left, resolution_left);
                } else {
                    PrintWarning("[UbitrackSingleCameraVisualizer] error retrieving intrinsics.\n");
                }

                // transfer camera_left_image to opengl texture
                if (connector->camera_left_get_current_image(ubitrack_camera_image_ptr)) {
                    // do we have a valid image?
                    if (ubitrack_camera_image_ptr && (ubitrack_camera_image_ptr->isValid())) {
                        if (!textureupdate.isInitialized()) {
                            textureupdate.initializeTexture(ubitrack_camera_image_ptr);
                        }

                        textureupdate.updateTexture(ubitrack_camera_image_ptr);
                    }
                } else {
                    PrintWarning("[UbitrackSingleCameraVisualizer] error retrieving camera image.\n");
                }

                Eigen::Matrix4d extrinsics_left;
                if(connector->camera_left_get_pose(ts, extrinsics_left)){
                    view_control.SetCameraExtrinsics(extrinsics_left);
                } else {
                    PrintWarning("[UbitrackSingleCameraVisualizer] error retrieving extrinsics.\n");
                }

                // things have changed, notify renderer
                UpdateWindowTitle();
                return true;

            } else {
                PrintWarning("[UbitrackSingleCameraVisualizer] no data was received from connector.\n");
            }

          UpdateWindowTitle();
          return false;
        });
}


} // three