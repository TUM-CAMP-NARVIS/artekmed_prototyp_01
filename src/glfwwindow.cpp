//
// Created by Ulrich Eck on 10/05/15.
//


#include "simple_ar_demo/glfwwindow.h"

Window::Window(int width, int height, const std::string& title) {
    // initialise glfw and m_glfwWindow,
    // create openGL context, initialise any other c++ resources
    glfwSetErrorCallback(ErrorCallback);
    glfwInit();

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);


    m_glfwWindow = glfwCreateWindow(width, height, title.c_str(), NULL, NULL);
    glfwMakeContextCurrent(m_glfwWindow);

    // needed for glfwGetUserPointer to work
    glfwSetWindowUserPointer(m_glfwWindow, this);

    // set our static functions as callbacks
    glfwSetFramebufferSizeCallback(m_glfwWindow, WindowResizeCallback);
    glfwSetWindowRefreshCallback(m_glfwWindow, WindowRefreshCallback);
    glfwSetWindowCloseCallback(m_glfwWindow, WindowCloseCallback);
}

void Window::on_window_size(int w, int h)
{
    LINFO << "Received Window Size Event " << w << "," << h;
    int fb_width, fb_height;
    glfwGetFramebufferSize(m_glfwWindow, &fb_width, &fb_height);
    glViewport(0, 0, fb_width, fb_height);
}

void Window::on_window_close()
{
}

void Window::on_keypress(int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE) {
        LINFO << "Received Window Close Event.";
        glfwSetWindowShouldClose(m_glfwWindow, GL_TRUE);
    }
}

void Window::on_render()
{
}