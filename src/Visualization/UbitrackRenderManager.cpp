//
// Created by netlabs on 26.01.19.
//


#include <utVisualization.h>
#include <utVision/OpenGLPlatform.h>

#include "artekmed/Visualization/UbitrackRenderManager.h"

#include <utVision/OpenCLManager.h>
#include <utUtil/TracingProvider.h>

#ifdef HAVE_OPENCL
#include <opencv2/core/ocl.hpp>
#ifdef __APPLE__
#include "OpenCL/cl_gl.h"
#else
#include "CL/cl_gl.h"
#endif
#endif


using namespace Ubitrack::Visualization;


GLFWWindowImpl::GLFWWindowImpl(int _width, int _height, const std::string &_title)
        : VirtualWindow(_width, _height, _title), m_pWindow(NULL)
{

}

GLFWWindowImpl::~GLFWWindowImpl() {

}

bool GLFWWindowImpl::is_valid() {
    // maybe more checks.
    return (m_pWindow != NULL ) && (!glfwWindowShouldClose(m_pWindow));
}

bool GLFWWindowImpl::create() {
    std::cout << "Create GLFW Window." << std::endl;

#ifdef HAVE_OPENCL
    // access OCL Manager and initialize if needed
    Vision::OpenCLManager& oclManager = Vision::OpenCLManager::singleton();
    if ((oclManager.isActive()) && (!oclManager.isInitialized()))
    {
        glfwMakeContextCurrent(m_pWindow);
        if (oclManager.isEnabled()) {
            oclManager.initializeOpenGL();
        }
        std::cout << "OCL Manager initialized: " << oclManager.isInitialized() << std::endl;
    }
#endif

    m_pWindow = glfwCreateWindow(m_width, m_height, m_title.c_str(), NULL, NULL);

    // set fullscreen ?
    return m_pWindow != NULL;
}

void GLFWWindowImpl::reshape(int w, int h) {
    VirtualWindow::reshape(w, h);
}


void GLFWWindowImpl::setFullscreen(bool fullscreen) {
    if (!m_pWindow) {
        return;
    }
#if (defined(GLFW_VERSION_MAJOR) && (GLFW_VERSION_MAJOR >= 3) && (GLFW_VERSION_MINOR >= 2))
    if (fullscreen) {
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);
        glfwSetWindowMonitor(m_pWindow, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
    }
    else {
        CameraHandle *cam = static_cast<CameraHandle*>(glfwGetWindowUserPointer(m_pWindow));
        glfwSetWindowMonitor(m_pWindow, NULL, 0, 0, cam->initial_width(), cam->initial_height(), 0);
    }
#else
    std::cout << "GLFW Version below 3.2 does not support switching to fullscreen during runtime" << std::endl;
#endif
}

void GLFWWindowImpl::onExit() {
    if (m_pWindow) {
        std::cout << "Request to close window." << std::endl;
        glfwSetWindowShouldClose(m_pWindow, 1);
    }
}

void GLFWWindowImpl::initGL(boost::shared_ptr<CameraHandle>& event_handler) {
    // log error ??
    if (m_pWindow == NULL)
        return;

    glfwMakeContextCurrent(m_pWindow);

// @todo we should make sure that not both libraries are active ...
#ifdef HAVE_GLAD
    // Init GLAD for this context:
    std::cout << "Initialize GLAD." << std::endl;
    if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress))
    {
        std::cout << "Failed to initialize OpenGL context" << std::endl;
        glfwDestroyWindow(m_pWindow);
        return;
    }
#endif
#ifdef HAVE_GLEW
    // Init GLEW for this context:
    std::cout << "Initialize GLEW." << std::endl;
    GLenum err = glewInit();
    if (err != GLEW_OK) {
        std::cout << "GLEW Error occured, Description: " <<  glewGetErrorString(err) << std::endl;
        glfwDestroyWindow(m_pWindow);
        return;
    }
#endif


    // GL: enable and set colors
    glEnable(GL_COLOR_MATERIAL);
    glClearColor(0.0, 0.0, 0.0, 1.0); // TODO: make this configurable (but black is best for optical see-through ar!)

    // GL: enable and set depth parameters
    glEnable(GL_DEPTH_TEST);
    glClearDepth(1.0);

    // GL: disable backface culling
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    glDisable(GL_CULL_FACE);

    // GL: light parameters
    GLfloat light_pos[] = {1.0f, 1.0f, 1.0f, 0.0f};
    GLfloat light_amb[] = {0.2f, 0.2f, 0.2f, 1.0f};
    GLfloat light_dif[] = {0.9f, 0.9f, 0.9f, 1.0f};

    // GL: enable lighting
    glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
    glLightfv(GL_LIGHT0, GL_AMBIENT, light_amb);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_dif);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    // GL: bitmap handling
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    // GL: alpha blending
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_BLEND);

    // GL: misc stuff
    glShadeModel(GL_SMOOTH);
    glEnable(GL_NORMALIZE);

    // setup callbacks:
    m_pEventHandler = event_handler;
    glfwSetWindowUserPointer(m_pWindow, event_handler.get());

    // which one is the better window-size callback in GLFW ??
    //glfwSetWindowSizeCallback(m_pWindow, WindowResizeCallback);
    glfwSetFramebufferSizeCallback(m_pWindow, WindowResizeCallback);
    glfwSetWindowRefreshCallback(m_pWindow, WindowRefreshCallback);
    glfwSetWindowCloseCallback(m_pWindow, WindowCloseCallback);
    glfwSetKeyCallback(m_pWindow, WindowKeyCallback);
    glfwSetCursorPosCallback(m_pWindow, WindowCursorPosCallback);

}

void GLFWWindowImpl::destroy() {
    if (m_pWindow != NULL) {
        glfwSetWindowUserPointer(m_pWindow, NULL);
        glfwDestroyWindow(m_pWindow);
        m_pWindow = NULL;
    }
    m_pEventHandler.reset();
}


void GLFWWindowImpl::pre_render() {
    glfwMakeContextCurrent(m_pWindow);
}

void GLFWWindowImpl::post_render() {
    glfwSwapBuffers(m_pWindow);
}
