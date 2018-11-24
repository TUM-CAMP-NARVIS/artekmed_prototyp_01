//
// Created by Ulrich Eck on 15.03.18.
//


#include "basic_facade_demo/UbitrackVisualizer.h"

#include <thread>

#include <Core/Utility/FileSystem.h>
#include <Core/Utility/Console.h>
#include <Core/Camera/PinholeCameraTrajectory.h>
#include <IO/ClassIO/IJsonConvertibleIO.h>

#include <utFacade/BasicFacade.h>
#include <basic_facade_demo/UbitrackViewControl.h>
#include <basic_facade_demo/UbitrackImageRenderer.h>

#include <log4cpp/Category.hh>
static log4cpp::Category& logger(log4cpp::Category::getInstance("BasicFacadeExample.UbitrackVisualizer"));

namespace open3d {

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


bool UbitrackVisualizer::AddUbitrackImage(std::shared_ptr<const UbitrackImage> geometry_ptr)
{
    if (is_initialized_ == false) {
        return false;
    }
    glfwMakeContextCurrent(window_);

    if (geometry_ptr->GetGeometryType() ==
            Geometry::GeometryType::Image) {
        auto renderer_ptr = std::make_shared<glsl::UbitrackImageRenderer>();
        if (renderer_ptr->AddGeometry(geometry_ptr) == false) {
            return false;
        }
        geometry_renderer_ptrs_.push_back(renderer_ptr);
    } else {
        return false;
    }

    geometry_ptrs_.push_back(geometry_ptr);
    view_control_ptr_->FitInGeometry(*geometry_ptr);
    ResetViewPoint();
    LOG4CPP_INFO(logger, "Add geometry and update bounding box to " << view_control_ptr_->GetBoundingBox().GetPrintInfo().c_str());
    return UpdateGeometry();
}

void UbitrackVisualizer::UpdateWindowTitle()
{
    if (window_ != nullptr) {
        auto view_control = dynamic_cast<UbitrackViewControl*>(view_control_ptr_.get());
        std::string new_window_title = window_name_ + " - " + view_control->GetStatusString();
        glfwSetWindowTitle(window_, new_window_title.c_str());
    }
}

bool UbitrackVisualizer::InitViewControl()
{
    view_control_ptr_ = std::unique_ptr<ViewControl>(new UbitrackViewControl);
    ResetViewPoint();
    return true;
}

bool UbitrackVisualizer::InitRenderOption()
{
    if(Visualizer::InitRenderOption()) {
        render_option_ptr_->image_stretch_option_ = RenderOption::ImageStretchOption::StretchWithWindow;
        return true;
    }
    return false;

}

void UbitrackVisualizer::WindowCloseCallback(GLFWwindow *window) {
    StopUbitrack();
    Visualizer::WindowCloseCallback(window);
}


void UbitrackVisualizer::Render()
{
    glfwMakeContextCurrent(window_);

    auto view_control = dynamic_cast<UbitrackViewControl*>(view_control_ptr_.get());
    view_control->SetUbitrackViewMatrices();

    glEnable(GL_MULTISAMPLE);
    glDisable(GL_BLEND);
    auto &background_color = render_option_ptr_->background_color_;
    glClearColor((GLclampf)background_color(0), (GLclampf)background_color(1),
            (GLclampf)background_color(2), 1.0f);
    glClearDepth(1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    for (const auto &renderer_ptr : geometry_renderer_ptrs_) {
        renderer_ptr->Render(*render_option_ptr_, *view_control_ptr_);
    }
    for (const auto &renderer_ptr : utility_renderer_ptrs_) {
        renderer_ptr->Render(*render_option_ptr_, *view_control_ptr_);
    }

    glfwSwapBuffers(window_);
}



void UbitrackVisualizer::KeyPressCallback(GLFWwindow *window,
        int key, int scancode, int action, int mods)
{
    auto view_control = dynamic_cast<UbitrackViewControl*>(view_control_ptr_.get());
    if (action == GLFW_RELEASE) {
        return;
    }

    if (mods & GLFW_MOD_CONTROL) {
        switch (key) {
        case GLFW_KEY_F:
            LOG4CPP_DEBUG(logger, "unhandled keypress.\n");
            break;
        case GLFW_KEY_W:
            LOG4CPP_DEBUG(logger, "unhandled keypress.\n");
            break;
        case GLFW_KEY_P:
            LOG4CPP_DEBUG(logger, "unhandled keypress.\n");
            break;
        case GLFW_KEY_R:
            LOG4CPP_DEBUG(logger, "unhandled keypress.\n");
            break;
        case GLFW_KEY_G:
            LOG4CPP_DEBUG(logger, "unhandled keypress.\n");
            break;
        case GLFW_KEY_S:
            LOG4CPP_DEBUG(logger, "unhandled keypress.\n");
            break;
        case GLFW_KEY_LEFT:
            LOG4CPP_DEBUG(logger, "unhandled keypress.\n");
            break;
        case GLFW_KEY_RIGHT:
            LOG4CPP_DEBUG(logger, "unhandled keypress.\n");
            break;
        case GLFW_KEY_LEFT_BRACKET:
            LOG4CPP_DEBUG(logger, "unhandled keypress.\n");
            break;
        case GLFW_KEY_RIGHT_BRACKET:
            LOG4CPP_DEBUG(logger, "unhandled keypress.\n");
            break;
        case GLFW_KEY_EQUAL:
            LOG4CPP_DEBUG(logger, "unhandled keypress.\n");
            break;
        case GLFW_KEY_MINUS:
            LOG4CPP_DEBUG(logger, "unhandled keypress.\n");
            break;
        case GLFW_KEY_L:
            LOG4CPP_DEBUG(logger, "unhandled keypress.\n");
            break;
        case GLFW_KEY_A:
            LOG4CPP_DEBUG(logger, "unhandled keypress.\n");
            break;
        case GLFW_KEY_U:
            LOG4CPP_DEBUG(logger, "unhandled keypress.\n");
            break;
        case GLFW_KEY_D:
            view_control->PrintDebugMatrices();
            break;
        case GLFW_KEY_N:
            LOG4CPP_DEBUG(logger, "unhandled keypress.\n");
            break;
        case GLFW_KEY_E:
            LOG4CPP_DEBUG(logger, "unhandled keypress.\n");
            break;
        default:
            Visualizer::KeyPressCallback(window, key, scancode, action, mods);
            break;
        }
        is_redraw_required_ = true;
        UpdateWindowTitle();
    } else {
        Visualizer::KeyPressCallback(window, key, scancode, action, mods);
    }
}

void UbitrackVisualizer::MouseMoveCallback(GLFWwindow* window,
        double x, double y)
{
//    auto view_control = dynamic_cast<UbitrackViewControl*>(view_control_ptr_.get());
    Visualizer::MouseMoveCallback(window, x, y);
}

void UbitrackVisualizer::MouseScrollCallback(GLFWwindow* window,
        double x, double y)
{
//    auto view_control = dynamic_cast<UbitrackViewControl*>(view_control_ptr_.get());
    Visualizer::MouseScrollCallback(window, x, y);
}

void UbitrackVisualizer::MouseButtonCallback(GLFWwindow* window,
        int button, int action, int mods)
{
//    auto view_control = dynamic_cast<UbitrackViewControl*>(view_control_ptr_.get());
    Visualizer::MouseButtonCallback(window, button, action, mods);
}

}	// namespace open3d
