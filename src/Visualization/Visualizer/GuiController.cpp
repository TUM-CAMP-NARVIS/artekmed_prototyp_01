//
// Created by netlabs on 26.01.19.
//

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

        ImGui_ImplGlfw_InitForOpenGL(window, false);
        ImGui_ImplOpenGL2_Init();

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
        ImGui_ImplOpenGL2_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
    }
}

void GuiController::render(const int width, const int height) {
    if (m_enabled) {
//        ImGui::Begin("Testing");
        ImGui::Text("Hello from GuiController!");
        if (ImGui::Button("Press Me")) {
            ///
        }
//        ImGui::End();
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
        ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

        glfwMakeContextCurrent(window);
        glEnable(GL_DEPTH_TEST);
    }
}