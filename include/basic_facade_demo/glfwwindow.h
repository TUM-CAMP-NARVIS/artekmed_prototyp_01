//
// Created by Ulrich Eck on 10/05/15.
//

#ifndef SIMPLE_AR_DEMO_GLFWWINDOW_H
#define SIMPLE_AR_DEMO_GLFWWINDOW_H


// Window.hpp
#include <string>
#include <GLFW/glfw3.h>



class Window {
public:
    Window(int width, int height, const std::string& title);

    // virtual callbacks for implementation
    virtual void on_window_size(int w, int h);
    virtual void on_window_close();
    virtual void on_keypress(int key, int scancode, int action, int mods);
    virtual void on_render();
    // add more methods for mouse handling


    // utilities
    bool windowShouldClose() {
        return (glfwWindowShouldClose(m_glfwWindow)!=0);
    }

    inline void swap() {
        glfwSwapBuffers(m_glfwWindow);
    }

private:
    GLFWwindow *m_glfwWindow;

    // callback implementations for GLFW
    inline static void WindowResizeCallback(
            GLFWwindow *win,
            int w,
            int h) {
        Window *window = static_cast<Window*>(glfwGetWindowUserPointer(win));
        window->on_window_size(w, h);
    }

    inline static void WindowRefreshCallback(GLFWwindow *win) {
        Window *window = static_cast<Window*>(glfwGetWindowUserPointer(win));
        window->on_render();
    }

    inline static void WindowCloseCallback(GLFWwindow *win) {
        Window *window = static_cast<Window*>(glfwGetWindowUserPointer(win));
        window->on_window_close();
    }

    inline static void WindowKeyCallback(GLFWwindow *win,
                                         int key,
                                         int scancode,
                                         int action,
                                         int mods) {
        Window *window = static_cast<Window*>(glfwGetWindowUserPointer(win));
        window->on_keypress(key, scancode, action, mods);
    }

    inline static void ErrorCallback(int error, const char* description) {
        // improve ...
        fputs(description, stderr);
    }

};

#endif //SIMPLE_AR_DEMO_GLFWWINDOW_H
