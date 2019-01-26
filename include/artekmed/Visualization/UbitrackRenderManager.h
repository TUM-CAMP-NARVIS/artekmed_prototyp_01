//
// Created by netlabs on 26.01.19.
//

#ifndef ARTEKMED_P1_UBITRACKRENDERMANAGER_H
#define ARTEKMED_P1_UBITRACKRENDERMANAGER_H


#include <utVisualization.h>

#include <utVision/OpenGLPlatform.h>
#include <utVisualization/utRenderAPI.h>

#include <string>
#include <GLFW/glfw3.h>

namespace Ubitrack {
    namespace Visualization {

        class GLFWWindowImpl : public VirtualWindow {

        public:
            GLFWWindowImpl(int _width, int _height, const std::string& _title);
            ~GLFWWindowImpl();

            virtual void pre_render();
            virtual void post_render();

            virtual void reshape(int w, int h);

            //custom extensions
            virtual void setFullscreen(bool fullscreen);
            virtual void onExit();


            // Implementation of Public interface
            virtual bool is_valid();
            virtual bool create();
            virtual void initGL(boost::shared_ptr<CameraHandle>& cam);
            virtual void destroy();

        private:
            GLFWwindow*	m_pWindow;
            boost::shared_ptr<CameraHandle> m_pEventHandler;
        };

        // callback implementations for GLFW
        inline static void WindowResizeCallback(
                GLFWwindow *win,
                int w,
                int h) {
            CameraHandle *cam = static_cast<CameraHandle*>(glfwGetWindowUserPointer(win));
            cam->on_window_size(w, h);
        }

        inline static void WindowRefreshCallback(GLFWwindow *win) {
            CameraHandle *cam = static_cast<CameraHandle*>(glfwGetWindowUserPointer(win));
            cam->on_render(glfwGetTime());
        }

        inline static void WindowCloseCallback(GLFWwindow *win) {
            CameraHandle *cam = static_cast<CameraHandle*>(glfwGetWindowUserPointer(win));
            cam->on_window_close();
        }

        inline static void WindowKeyCallback(GLFWwindow *win,
                                             int key,
                                             int scancode,
                                             int action,
                                             int mods) {
            CameraHandle *cam = static_cast<CameraHandle*>(glfwGetWindowUserPointer(win));
            if ((action == GLFW_PRESS) && (mods & GLFW_MOD_ALT)) {
                switch (key) {
                    case GLFW_KEY_F:
                        cam->on_fullscreen();
                        return;
                    default:
                        break;
                }
            }
            if (action == GLFW_PRESS) {
                switch (key) {
                    case GLFW_KEY_ESCAPE:
                        cam->on_exit();
                        return;
                    default:
                        cam->on_keypress(key, scancode, action, mods);
                        break;
                }
            }
        }


        inline static void WindowCursorPosCallback(GLFWwindow *win,
                                                   double xpos,
                                                   double ypos) {
            CameraHandle *cam = static_cast<CameraHandle*>(glfwGetWindowUserPointer(win));
            cam->on_cursorpos(xpos, ypos);
        }

    }
}

#endif //ARTEKMED_P1_UBITRACKRENDERMANAGER_H
