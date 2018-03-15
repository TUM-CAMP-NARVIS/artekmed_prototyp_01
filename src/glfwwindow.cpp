//
// Created by Ulrich Eck on 10/05/15.
//

#include "basic_facade_demo/glfwwindow.h"

#include <log4cpp/Category.hh>
static log4cpp::Category& logger(log4cpp::Category::getInstance("BasicFacadeExample.glfwwindow"));


Window::Window(int width, int height, const std::string& title) {
    // initialise glfw and m_glfwWindow,
    // create openGL context, initialise any other c++ resources
    glfwSetErrorCallback(ErrorCallback);
    glfwInit();

	// select OpenGL Version
	glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
//    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
//    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    // oldschool
    //glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    //glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

    glfwWindowHint(GLFW_RESIZABLE, GL_TRUE);
	//glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, 1); 

    m_glfwWindow = glfwCreateWindow(width, height, title.c_str(), NULL, NULL);
    glfwMakeContextCurrent(m_glfwWindow);

    // needed for glfwGetUserPointer to work
    glfwSetWindowUserPointer(m_glfwWindow, this);

    // set our static functions as callbacks
    glfwSetFramebufferSizeCallback(m_glfwWindow, WindowResizeCallback);
    glfwSetWindowRefreshCallback(m_glfwWindow, WindowRefreshCallback);
    glfwSetWindowCloseCallback(m_glfwWindow, WindowCloseCallback);

    // should not be needed ...
    on_window_size(1280, 480);
}

void Window::on_window_size(int w, int h)
{
    int fb_width, fb_height;
    glfwGetFramebufferSize(m_glfwWindow, &fb_width, &fb_height);
    LOG4CPP_INFO( logger, "Received Window Size Event " << w << "," << h << " framebuffer: " << fb_width << "," << fb_height);
    glViewport(0, 0, fb_width, fb_height);
}

void Window::on_window_close()
{
}


void Window::on_keypress(int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE) {
		LOG4CPP_INFO(logger, "Received Window Close Event.");
        glfwSetWindowShouldClose(m_glfwWindow, GL_TRUE);
    }
}

void Window::on_render()
{
}