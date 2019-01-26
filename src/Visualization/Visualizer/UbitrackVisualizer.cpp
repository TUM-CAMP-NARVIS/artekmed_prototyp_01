//
// Created by Ulrich Eck on 15.03.18.
//


#include "artekmed/Visualization/Visualizer/UbitrackVisualizer.h"

#include <thread>

#include <Core/Utility/FileSystem.h>
#include <Core/Utility/Console.h>
#include <Core/Camera/PinholeCameraTrajectory.h>
#include <IO/ClassIO/IJsonConvertibleIO.h>

#include <utFacade/BasicFacade.h>
#include <artekmed/Visualization/Visualizer/UbitrackViewControl.h>
#include <artekmed/Visualization/Shader/GeometryRenderer.h>

#include <log4cpp/Category.hh>
static log4cpp::Category& logger(log4cpp::Category::getInstance("ArtekmedP1.UbitrackVisualizer"));

namespace artekmed {

UbitrackVisualizer::UbitrackVisualizer()
{
}

UbitrackVisualizer::~UbitrackVisualizer()
{
}

bool UbitrackVisualizer::InitOpenGL()
{
    // Init GLAD for this context:
    LOG4CPP_INFO(logger, "Initialize GLEW.");
    if (glewInit() != GLEW_OK) {
        LOG4CPP_ERROR(logger, "Failed to initialize GLEW.");
        return false;
    }

    // depth test
    glEnable(GL_DEPTH_TEST);
    glClearDepth(1.0f);

    // pixel alignment
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    // polygon rendering
    glEnable(GL_CULL_FACE);

    // glReadPixels always read front buffer
    glReadBuffer(GL_FRONT);

    return true;
}


void UbitrackVisualizer::PrintVisualizerHelp()
{
    Visualizer::PrintVisualizerHelp();
    PrintInfo("    -- AR Mode --\n");
    PrintInfo("    XXX : Do something.\n");
    PrintInfo("\n");
}

void UbitrackVisualizer::UpdateWindowTitle()
{
    if (window_ != nullptr) {
        auto view_control = dynamic_cast<UbitrackViewControl*>(view_control_ptr_.get());
        std::string new_window_title = window_name_;
        glfwSetWindowTitle(window_, new_window_title.c_str());
    }
}

bool UbitrackVisualizer::InitViewControl()
{
    view_control_ptr_ = std::unique_ptr<ViewControl>(new UbitrackViewControl);
    ResetViewPoint();
    return true;
}

void UbitrackVisualizer::WindowCloseCallback(GLFWwindow *window) {
    StopUbitrack();
    Visualizer::WindowCloseCallback(window);
}

}	// namespace open3d
