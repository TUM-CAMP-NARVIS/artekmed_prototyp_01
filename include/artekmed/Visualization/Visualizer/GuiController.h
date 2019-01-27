//
// Created by netlabs on 26.01.19.
//

#ifndef ARTEKMED_P1_GUICONTROLLER_H
#define ARTEKMED_P1_GUICONTROLLER_H

#include <string>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include "artekmed/Visualization/Visualizer/ViewControl.h"

namespace artekmed {

//    using namespace open3d;

    class Visualizer;

    class GuiController {

    public:
        GuiController();

        void enabled(bool v) {
            m_enabled = v;
        }
        bool enabled() {
            return m_enabled;
        }

        virtual void initialize(GLFWwindow* window);

        void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
        void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset);
        void KeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
        void CharCallback(GLFWwindow* window, unsigned int c);


        bool addCheckBox(const char label[], bool& var, bool sameLine);
        bool addCheckBox(const char label[], int& var, bool sameLine);
        bool addCheckboxes(const char label[], bool* pData, uint32_t numCheckboxes, bool sameLine);

        virtual void pre_render();
        virtual void render(const int width, const int height, Visualizer* viewcontrol);
        virtual void post_render(GLFWwindow* window);

    protected:
        bool m_enabled = false;

    };

}


#endif //ARTEKMED_P1_GUICONTROLLER_H
