//
// Created by netlabs on 26.01.19.
//

#include "artekmed/Visualization/Visualizer/Visualizer.h"
#include "artekmed/Visualization/Visualizer/GuiController.h"


#include <GLFW/glfw3.h>

using namespace artekmed;

GuiController::GuiController() {

}

void GuiController::initialize(GLFWwindow* window) {
    if (m_enabled) {
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();

        ImGuiIO& io = ImGui::GetIO(); (void)io;
        //io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;  // Enable Keyboard Controls
        //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;   // Enable Gamepad Controls

        ImGui_ImplGlfw_InitForOpenGL(window, true);
        ImGui_ImplOpenGL3_Init();

        ImGui::StyleColorsDark();
    }
}


void GuiController::MouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    if (m_enabled) {
        ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mods);
    }
};


void GuiController::ScrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    if (m_enabled) {
        ImGui_ImplGlfw_ScrollCallback(window, xoffset, yoffset);
    }
};


void GuiController::KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (m_enabled) {
        ImGui_ImplGlfw_KeyCallback(window, key, scancode, action, mods);
    }
};


void GuiController::CharCallback(GLFWwindow* window, unsigned int c) {
    if (m_enabled) {
        ImGui_ImplGlfw_CharCallback(window, c);
    }
};

void GuiController::pre_render() {
    if (m_enabled) {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
    }
}

bool GuiController::addCheckBox(const char label[], bool& var, bool sameLine)
{
    if (sameLine) ImGui::SameLine();
    return ImGui::Checkbox(label, &var);
}

bool GuiController::addCheckBox(const char label[], int& var, bool sameLine)
{
    bool value = (var != 0);
    bool modified = addCheckBox(label, value, sameLine);
    var = (value ? 1 : 0);
    return modified;
}

bool GuiController::addCheckboxes(const char label[], bool* pData, uint32_t numCheckboxes, bool sameLine)
{
    bool modified = false;
    std::string labelString(std::string("##") + label + '0');

    for (uint32_t i = 0; i < numCheckboxes - 1; ++i)
    {
        labelString[labelString.size() - 1] = '0' + static_cast<int32_t>(i);
        modified |= addCheckBox(labelString.c_str(), pData[i], (!i) ? sameLine : true);
    }

    addCheckBox(label, pData[numCheckboxes - 1], true );

    return modified;
}

void GuiController::render(const int width, const int height, Visualizer* vis) {
    if (m_enabled) {
        ImGui::Begin("Display");

        std::vector<char>& arr = vis->GetGeometryFlags();
        // not a nice solution .. but std::vector stores bit-streams ..
        bool* checkbox_arr = (bool*)malloc(arr.size()*sizeof(bool));
        for (int i = 0; i < arr.size(); ++i) {
            if (arr[i] == 0x00) {
                checkbox_arr[i] = false;
            } else {
                checkbox_arr[i] = true;
            }
        }
        addCheckboxes("Geometries", checkbox_arr, arr.size(), true);
        for (int i = 0; i < arr.size(); ++i) {
            if (checkbox_arr[i]) {
                arr[i] = 0x01;
            } else {
                arr[i] = 0x00;
            }
        }
        // free manually allocated buffer
        free(checkbox_arr);

        ImGui::End();

        ImGui::Begin("Open3D");

        ImGui::Text("Camera FOV: %.2f", vis->GetViewControl().GetFieldOfView());
        if (ImGui::Button("FieldOfView+")) {
            vis->GetViewControl().ChangeFieldOfView(1.0);
        }
        if (ImGui::Button("FieldOfView-")) {
            vis->GetViewControl().ChangeFieldOfView(-1.0);
        }
        if (ImGui::Button("Reset Viewpoint")) {
            vis->GetViewControl().Reset();
        }

        ImGui::Text("Capture");
        if (ImGui::Button("Capture ScreenImage")) {
            vis->CaptureScreenImage();
        }
        if (ImGui::Button("Capture DepthImage")) {
            vis->CaptureDepthImage();
        }
        if (ImGui::Button("Capture RenderOption")) {
            vis->CaptureRenderOption();
        }

        ImGui::Text("Rendering");
        if (ImGui::Button("Toggle Light")) {
            vis->GetRenderOption().ToggleLightOn();
        }

        ImGui::Text("Point size %.2f.", vis->GetRenderOption().point_size_);
        if (ImGui::Button("PointSize+")) {
            vis->GetRenderOption().ChangePointSize(1.0);
        }
        if (ImGui::Button("PointSize-")) {
            vis->GetRenderOption().ChangePointSize(-1.0);
        }
        if (ImGui::Button("Toggle PointShowNormal")) {
            vis->GetRenderOption().TogglePointShowNormal();
            if (vis->GetRenderOption().point_show_normal_) {
                vis->UpdateGeometry();
            }
        }
        // render_option_ptr_->mesh_shade_option_ ==
        //                   RenderOption::MeshShadeOption::FlatShade ?
        //                   "FLAT" : "SMOOTH")
        if (ImGui::Button("Toggle MeshShowWireframe")) {
            vis->GetRenderOption().ToggleShadingOption();
            vis->UpdateGeometry();
        }

        // render_option_ptr_->mesh_show_wireframe_ ? "ON" : "OFF"
        if (ImGui::Button("Toggle ShadingOption")) {
            vis->GetRenderOption().ToggleMeshShowWireframe();
            vis->UpdateGeometry();
        }

        // render_option_ptr_->mesh_show_back_face_ ? "ON" : "OFF"
        if (ImGui::Button("Toggle MeshShowBackFace")) {
            vis->GetRenderOption().ToggleMeshShowBackFace();
        }

        // render_option_ptr_->interpolation_option_ ==
        //                   RenderOption::TextureInterpolationOption::Nearest ?
        //                   "NEARST" : "LINEAR"
        if (ImGui::Button("Toggle InterpolationOption")) {
            vis->GetRenderOption().ToggleInterpolationOption();
            vis->UpdateGeometry();
        }


        // todo ...
//        case GLFW_KEY_T:
//            render_option_ptr_->ToggleImageStretchOption();
//        PrintDebug("[Visualizer] Image stretch mode is #%d.\n",
//                   int(render_option_ptr_->image_stretch_option_));
//        break;
//        case GLFW_KEY_0:
//            if (mods & GLFW_MOD_CONTROL) {
//                render_option_ptr_->mesh_color_option_ =
//                        RenderOption::MeshColorOption::Default;
//                UpdateGeometry();
//                PrintDebug("[Visualizer] Mesh color set to DEFAULT.\n");
//            } else if (mods & GLFW_MOD_SHIFT) {
//                SetGlobalColorMap(ColorMap::ColorMapOption::Gray);
//                UpdateGeometry();
//                PrintDebug("[Visualizer] Color map set to GRAY.\n");
//            } else {
//                render_option_ptr_->point_color_option_ =
//                        RenderOption::PointColorOption::Default;
//                UpdateGeometry();
//                PrintDebug("[Visualizer] Point color set to DEFAULT.\n");
//            }
//        break;
//        case GLFW_KEY_1:
//            if (mods & GLFW_MOD_CONTROL) {
//                render_option_ptr_->mesh_color_option_ =
//                        RenderOption::MeshColorOption::Color;
//                UpdateGeometry();
//                PrintDebug("[Visualizer] Mesh color set to COLOR.\n");
//            } else if (mods & GLFW_MOD_SHIFT) {
//                SetGlobalColorMap(ColorMap::ColorMapOption::Jet);
//                UpdateGeometry();
//                PrintDebug("[Visualizer] Color map set to JET.\n");
//            } else {
//                render_option_ptr_->point_color_option_ =
//                        RenderOption::PointColorOption::Color;
//                UpdateGeometry();
//                PrintDebug("[Visualizer] Point color set to COLOR.\n");
//            }
//        break;
//        case GLFW_KEY_2:
//            if (mods & GLFW_MOD_CONTROL) {
//                render_option_ptr_->mesh_color_option_ =
//                        RenderOption::MeshColorOption::XCoordinate;
//                UpdateGeometry();
//                PrintDebug("[Visualizer] Mesh color set to X.\n");
//            } else if (mods & GLFW_MOD_SHIFT) {
//                SetGlobalColorMap(ColorMap::ColorMapOption::Summer);
//                UpdateGeometry();
//                PrintDebug("[Visualizer] Color map set to SUMMER.\n");
//            } else {
//                render_option_ptr_->point_color_option_ =
//                        RenderOption::PointColorOption::XCoordinate;
//                UpdateGeometry();
//                PrintDebug("[Visualizer] Point color set to X.\n");
//            }
//        break;
//        case GLFW_KEY_3:
//            if (mods & GLFW_MOD_CONTROL) {
//                render_option_ptr_->mesh_color_option_ =
//                        RenderOption::MeshColorOption::YCoordinate;
//                UpdateGeometry();
//                PrintDebug("[Visualizer] Mesh color set to Y.\n");
//            } else if (mods & GLFW_MOD_SHIFT) {
//                SetGlobalColorMap(ColorMap::ColorMapOption::Winter);
//                UpdateGeometry();
//                PrintDebug("[Visualizer] Color map set to WINTER.\n");
//            } else {
//                render_option_ptr_->point_color_option_ =
//                        RenderOption::PointColorOption::YCoordinate;
//                UpdateGeometry();
//                PrintDebug("[Visualizer] Point color set to Y.\n");
//            }
//        break;
//        case GLFW_KEY_4:
//            if (mods & GLFW_MOD_CONTROL) {
//                render_option_ptr_->mesh_color_option_ =
//                        RenderOption::MeshColorOption::ZCoordinate;
//                UpdateGeometry();
//                PrintDebug("[Visualizer] Mesh color set to Z.\n");
//            } else if (mods & GLFW_MOD_SHIFT) {
//                SetGlobalColorMap(ColorMap::ColorMapOption::Hot);
//                UpdateGeometry();
//                PrintDebug("[Visualizer] Color map set to HOT.\n");
//            } else {
//                render_option_ptr_->point_color_option_ =
//                        RenderOption::PointColorOption::ZCoordinate;
//                UpdateGeometry();
//                PrintDebug("[Visualizer] Point color set to Z.\n");
//            }
//        break;
//        case GLFW_KEY_9:
//            if (mods & GLFW_MOD_CONTROL) {
//                render_option_ptr_->mesh_color_option_ =
//                        RenderOption::MeshColorOption::Normal;
//                UpdateGeometry();
//                PrintDebug("[Visualizer] Mesh color set to NORMAL.\n");
//            } else if (mods & GLFW_MOD_SHIFT) {
//            } else {
//                render_option_ptr_->point_color_option_ =
//                        RenderOption::PointColorOption::Normal;
//                UpdateGeometry();
//                PrintDebug("[Visualizer] Point color set to NORMAL.\n");
//            }
//        break;

        ImGui::End();
    }
}

void GuiController::post_render(GLFWwindow* window) {
    if (m_enabled) {

        ImGui::Render();

        int display_w, display_h;
        glfwMakeContextCurrent(window);
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glDisable(GL_DEPTH_TEST);

//        glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
//        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwMakeContextCurrent(window);
        glEnable(GL_DEPTH_TEST);
    }
}