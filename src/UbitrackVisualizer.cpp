//
// Created by Ulrich Eck on 15.03.18.
//


#include "basic_facade_demo/UbitrackVisualizer.h"

#include <thread>

#include <Core/Utility/FileSystem.h>
#include <Core/Utility/Console.h>
#include <Core/Camera/PinholeCameraTrajectory.h>
#include <basic_facade_demo/UbitrackViewControl.h>
#include <IO/ClassIO/IJsonConvertibleIO.h>

namespace three{

UbitrackVisualizer::UbitrackVisualizer()
{
}

UbitrackVisualizer::~UbitrackVisualizer()
{
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
        auto &view_control = (UbitrackViewControl &)
                (*view_control_ptr_);
        std::string new_window_title = window_name_ + " - " +
                view_control.GetStatusString();
        glfwSetWindowTitle(window_, new_window_title.c_str());
    }
}

bool UbitrackVisualizer::InitViewControl()
{
    view_control_ptr_ = std::unique_ptr<UbitrackViewControl>(
            new UbitrackViewControl);
    ResetViewPoint();
    return true;
}

void UbitrackVisualizer::KeyPressCallback(GLFWwindow *window,
        int key, int scancode, int action, int mods)
{
    auto &view_control = (UbitrackViewControl &)(*view_control_ptr_);
    if (action == GLFW_RELEASE) {
        return;
    }

    if (mods & GLFW_MOD_CONTROL) {
        switch (key) {
        case GLFW_KEY_F:
            PrintDebug("[Visualizer] unhandled keypress.\n");
            break;
        case GLFW_KEY_W:
            PrintDebug("[Visualizer] unhandled keypress.\n");
            break;
        case GLFW_KEY_P:
            PrintDebug("[Visualizer] unhandled keypress.\n");
            break;
        case GLFW_KEY_R:
            PrintDebug("[Visualizer] unhandled keypress.\n");
            break;
        case GLFW_KEY_G:
            PrintDebug("[Visualizer] unhandled keypress.\n");
            break;
        case GLFW_KEY_S:
            PrintDebug("[Visualizer] unhandled keypress.\n");
            break;
        case GLFW_KEY_LEFT:
            PrintDebug("[Visualizer] unhandled keypress.\n");
            break;
        case GLFW_KEY_RIGHT:
            PrintDebug("[Visualizer] unhandled keypress.\n");
            break;
        case GLFW_KEY_LEFT_BRACKET:
            PrintDebug("[Visualizer] unhandled keypress.\n");
            break;
        case GLFW_KEY_RIGHT_BRACKET:
            PrintDebug("[Visualizer] unhandled keypress.\n");
            break;
        case GLFW_KEY_EQUAL:
            PrintDebug("[Visualizer] unhandled keypress.\n");
            break;
        case GLFW_KEY_MINUS:
            PrintDebug("[Visualizer] unhandled keypress.\n");
            break;
        case GLFW_KEY_L:
            PrintDebug("[Visualizer] unhandled keypress.\n");
            break;
        case GLFW_KEY_A:
            PrintDebug("[Visualizer] unhandled keypress.\n");
            break;
        case GLFW_KEY_U:
            PrintDebug("[Visualizer] unhandled keypress.\n");
            break;
        case GLFW_KEY_D:
            PrintDebug("[Visualizer] unhandled keypress.\n");
            break;
        case GLFW_KEY_N:
            PrintDebug("[Visualizer] unhandled keypress.\n");
            break;
        case GLFW_KEY_E:
            PrintDebug("[Visualizer] unhandled keypress.\n");
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
    auto &view_control = (UbitrackViewControl &)(*view_control_ptr_);
    Visualizer::MouseMoveCallback(window, x, y);
}

void UbitrackVisualizer::MouseScrollCallback(GLFWwindow* window,
        double x, double y)
{
    auto &view_control = (UbitrackViewControl &)(*view_control_ptr_);
    Visualizer::MouseScrollCallback(window, x, y);
}

void UbitrackVisualizer::MouseButtonCallback(GLFWwindow* window,
        int button, int action, int mods)
{
    auto &view_control = (UbitrackViewControl &)(*view_control_ptr_);
    Visualizer::MouseButtonCallback(window, button, action, mods);
}

}	// namespace three
