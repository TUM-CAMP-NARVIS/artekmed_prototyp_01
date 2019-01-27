//
// Created by netlabs on 26.01.19.
//

#ifndef ARTEKMED_P1_GUICONTROLLER_H
#define ARTEKMED_P1_GUICONTROLLER_H

#include <string>

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl2.h"


namespace artekmed {

//    using namespace open3d;

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

        virtual void pre_render();
        virtual void render(const int width, const int height);
        virtual void post_render(GLFWwindow* window);

    protected:
        bool m_enabled = false;

    };

}


#endif //ARTEKMED_P1_GUICONTROLLER_H
