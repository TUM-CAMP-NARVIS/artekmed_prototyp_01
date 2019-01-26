// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2018 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#include "artekmed/Visualization/Visualizer/Visualizer.h"

#include <Core/Geometry/TriangleMesh.h>

namespace artekmed {

namespace {

class GLFWEnvironmentSingleton
{
private:
    GLFWEnvironmentSingleton() { PrintDebug("GLFW init.\n");}
    GLFWEnvironmentSingleton(const GLFWEnvironmentSingleton &) = delete;
    GLFWEnvironmentSingleton &operator=(const GLFWEnvironmentSingleton &) =
            delete;
public:
    ~GLFWEnvironmentSingleton() {
        glfwTerminate();
        PrintDebug("GLFW destruct.\n");
    }

public:
    static GLFWEnvironmentSingleton &GetInstance() {
        static GLFWEnvironmentSingleton singleton;
        return singleton;
    }

    static int InitGLFW() {
        GLFWEnvironmentSingleton::GetInstance();
        return glfwInit();
    }

    static void GLFWErrorCallback(int error, const char* description) {
        PrintError("GLFW Error: %s\n", description);
    }
};

}    // unnamed namespace

Visualizer::Visualizer()
{
}

Visualizer::~Visualizer()
{
    glfwTerminate();    // to be safe
}

bool Visualizer::InitGLFW() {
    if (!GLFWEnvironmentSingleton::InitGLFW()) {
        PrintError("Failed to initialize GLFW\n");
        return false;
    }
    return true;
}

bool Visualizer::CreateVisualizerWindow(const std::string &window_name/* = "Open3D"*/,
        const int width/* = 640*/, const int height/* = 480*/,
        const int left/* = 50*/, const int top/* = 50*/, const bool visible/* = true*/)
{
    window_name_ = window_name;
    if (window_) {    // window already created
        UpdateWindowTitle();
        glfwSetWindowPos(window_, left, top);
        glfwSetWindowSize(window_, width, height);
#ifdef __APPLE__
        glfwSetWindowSize(window_,
                std::round(width * pixel_to_screen_coordinate_),
                std::round(height * pixel_to_screen_coordinate_));
        glfwSetWindowPos(window_,
                std::round(left * pixel_to_screen_coordinate_),
                std::round(top * pixel_to_screen_coordinate_));
#endif //__APPLE__
        return true;
    }

    glfwSetErrorCallback(GLFWEnvironmentSingleton::GLFWErrorCallback);
    if (!InitGLFW()) {
        return false;
    }

    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    glfwWindowHint(GLFW_VISIBLE, visible ? 1 : 0);

    window_ = glfwCreateWindow(width, height, window_name_.c_str(), NULL, NULL);
    if (!window_) {
        PrintError("Failed to create window\n");
        return false;
    }
    glfwSetWindowPos(window_, left, top);
    glfwSetWindowUserPointer(window_, this);

#ifdef __APPLE__
    // Some hacks to get pixel_to_screen_coordinate_
    glfwSetWindowSize(window_, 100, 100);
    glfwSetWindowPos(window_, 100, 100);
    int pixel_width_in_osx, pixel_height_in_osx;
    glfwGetFramebufferSize(window_, &pixel_width_in_osx, &pixel_height_in_osx);
    if (pixel_width_in_osx > 0) {
        pixel_to_screen_coordinate_ = 100.0 / (double)pixel_width_in_osx;
    } else {
        pixel_to_screen_coordinate_ = 1.0;
    }
    glfwSetWindowSize(window_,
            std::round(width * pixel_to_screen_coordinate_),
            std::round(height * pixel_to_screen_coordinate_));
    glfwSetWindowPos(window_,
            std::round(left * pixel_to_screen_coordinate_),
            std::round(top * pixel_to_screen_coordinate_));
#endif //__APPLE__

    auto window_refresh_callback = [](GLFWwindow *window) {
        static_cast<Visualizer *>(glfwGetWindowUserPointer(window))->
                WindowRefreshCallback(window);
    };
    glfwSetWindowRefreshCallback(window_, window_refresh_callback);

    auto window_resize_callback = [](GLFWwindow *window, int w, int h) {
        static_cast<Visualizer *>(glfwGetWindowUserPointer(window))->
                WindowResizeCallback(window, w, h);
    };
    glfwSetFramebufferSizeCallback(window_, window_resize_callback);

    auto mouse_move_callback = [](GLFWwindow *window, double x, double y) {
        static_cast<Visualizer *>(glfwGetWindowUserPointer(window))->
                MouseMoveCallback(window, x, y);
    };
    glfwSetCursorPosCallback(window_, mouse_move_callback);

    auto mouse_scroll_callback = [](GLFWwindow *window, double x, double y) {
        static_cast<Visualizer *>(glfwGetWindowUserPointer(window))->
                MouseScrollCallback(window, x, y);
    };
    glfwSetScrollCallback(window_, mouse_scroll_callback);

    auto mouse_button_callback = [](GLFWwindow *window,
            int button, int action, int mods)
    {
        static_cast<Visualizer *>(glfwGetWindowUserPointer(window))->
                MouseButtonCallback(window, button, action, mods);
    };
    glfwSetMouseButtonCallback(window_, mouse_button_callback);

    auto key_press_callback = [](GLFWwindow *window,
            int key, int scancode, int action, int mods)
    {
        static_cast<Visualizer *>(glfwGetWindowUserPointer(window))->
                KeyPressCallback(window, key, scancode, action, mods);
    };
    glfwSetKeyCallback(window_, key_press_callback);

    auto window_close_callback = [](GLFWwindow *window) {
        static_cast<Visualizer *>(glfwGetWindowUserPointer(window))->
                WindowCloseCallback(window);
    };
    glfwSetWindowCloseCallback(window_, window_close_callback);

    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    if (InitOpenGL() == false) {
        return false;
    }

    if (InitViewControl() == false) {
        return false;
    }

    if (InitRenderOption() == false) {
        return false;
    }

    int window_width, window_height;
    glfwGetFramebufferSize(window_, &window_width, &window_height);
    WindowResizeCallback(window_, window_width, window_height);

    UpdateWindowTitle();

    is_initialized_ = true;
    return true;
}

void Visualizer::DestroyVisualizerWindow()
{
    is_initialized_ = false;
    glfwDestroyWindow(window_);
    GetRenderManager().teardown();

}

void Visualizer::RegisterAnimationCallback(
        std::function<bool (Visualizer *)> callback_func)
{
    animation_callback_func_ = callback_func;
}

bool Visualizer::InitViewControl()
{
    view_control_ptr_ = std::unique_ptr<ViewControl>(new ViewControl);
    ResetViewPoint();
    return true;
}

bool Visualizer::InitRenderOption()
{
    render_option_ptr_ = std::unique_ptr<RenderOption>(new RenderOption);
    render_option_ptr_->image_stretch_option_ = RenderOption::ImageStretchOption::StretchWithWindow;
    return true;
}


bool Visualizer::SetupRenderManager() {
    // create and register render manager
    GetRenderManager().setup();
    is_renderermanager_initialized = true;
    return true;
}


void Visualizer::RenderManagerStep( Ubitrack::Visualization::RenderManager& manager) {
        boost::shared_ptr<Ubitrack::Visualization::CameraHandle> cam;
        boost::shared_ptr<Ubitrack::Visualization::GLFWWindowImpl> win;
        while (manager.need_setup()) {
            cam = manager.setup_pop_front();
            PrintDebug("Camera setup: %s\n", cam->title());
            win.reset(new Ubitrack::Visualization::GLFWWindowImpl(cam->initial_width(),
                                         cam->initial_height(),
                                         cam->title()));

            // XXX can this be simplified ??
            boost::shared_ptr<Ubitrack::Visualization::VirtualWindow> win_ = boost::dynamic_pointer_cast<Ubitrack::Visualization::VirtualWindow>(win);
            if (!cam->setup(win_)) {
                manager.setup_push_back(cam);
            } else {
                win->initGL(cam);
#ifdef WIN32
                Util::sleep(30);
#endif
//                windows_opened++;
            }
            glfwPollEvents();
        }

    std::vector< unsigned int > chToDelete;
    auto pos = manager.cameras_begin();
    auto end = manager.cameras_end();
    int ellapsed_time = (int)(glfwGetTime() * 1000.);
    while ( pos != end ) {
        bool is_valid = false;
        if (pos->second) {
            cam = pos->second;
            if (cam->get_window()) {
                win = boost::dynamic_pointer_cast<Ubitrack::Visualization::GLFWWindowImpl>(cam->get_window());
                if ((win) && (win->is_valid())) {
                    win->pre_render();
//							cam->pre_render();
                    cam->render(ellapsed_time);
                    //cam->post_render(); ??
                    is_valid = true;
                    win->post_render();  // make this loop through all current windows??
                    //CheckForGLErrors("Render Error");
                }
            }
        }
        if (!is_valid) {
            chToDelete.push_back(pos->first);
        }
        pos++;
        glfwPollEvents();
    }

    if (!chToDelete.empty()) {
        for (unsigned int i = 0; i < chToDelete.size(); i++) {
            unsigned int cam_id = chToDelete.at(i);
            manager.get_camera(cam_id)->teardown();
            manager.unregister_camera(cam_id);
            glfwPollEvents();
        }
    }
    // need a way to exit the loop here ..
    manager.wait_for_event(1);

}

void Visualizer::UpdateWindowTitle()
{
    if (window_ != NULL) {
        glfwSetWindowTitle(window_, window_name_.c_str());
    }
}

void Visualizer::BuildUtilities()
{
    glfwMakeContextCurrent(window_);

    // 0. Build coordinate frame
    const auto boundingbox = GetViewControl().GetBoundingBox();
    coordinate_frame_mesh_ptr_ = CreateMeshCoordinateFrame(
            boundingbox.GetSize() * 0.2, boundingbox.min_bound_);
    coordinate_frame_mesh_renderer_ptr_ =
            std::make_shared<glsl::CoordinateFrameRenderer>();
    if (coordinate_frame_mesh_renderer_ptr_->AddGeometry(
            coordinate_frame_mesh_ptr_) == false) {
        return;
    }
    utility_ptrs_.push_back(coordinate_frame_mesh_ptr_);
    utility_renderer_ptrs_.push_back(coordinate_frame_mesh_renderer_ptr_);
}

void Visualizer::Run()
{
    Ubitrack::Visualization::RenderManager& utrender = GetRenderManager();
    BuildUtilities();
    UpdateWindowTitle();
    while (bool(animation_callback_func_) ? PollEvents() : WaitEvents()) {
        RenderManagerStep(utrender);
        if (bool(animation_callback_func_in_loop_)) {
            if (animation_callback_func_in_loop_(this)) {
                UpdateGeometry();
            }
            // Set render flag as dirty anyways, because when we use callback
            // functions, we assume something has been changed in the callback
            // and the redraw event should be triggered.
            UpdateRender();
        }
    }
}

void Visualizer::Close()
{
    glfwSetWindowShouldClose(window_, GL_TRUE);
    PrintDebug("[Visualizer] Window closing.\n");
}

bool Visualizer::WaitEvents()
{
    if (is_initialized_ == false) {
        return false;
    }
    glfwMakeContextCurrent(window_);
    if (is_redraw_required_) {
        WindowRefreshCallback(window_);
    }
    animation_callback_func_in_loop_ = animation_callback_func_;
    glfwWaitEvents();
    return !glfwWindowShouldClose(window_);
}

bool Visualizer::PollEvents()
{
    if (is_initialized_ == false) {
        return false;
    }
    glfwMakeContextCurrent(window_);
    if (is_redraw_required_) {
        WindowRefreshCallback(window_);
    }
    animation_callback_func_in_loop_ = animation_callback_func_;
    glfwPollEvents();
    return !glfwWindowShouldClose(window_);
}

bool Visualizer::AddGeometry(std::shared_ptr<const Geometry> geometry_ptr)
{
    if (is_initialized_ == false) {
        return false;
    }

    glfwMakeContextCurrent(window_);
    if (geometry_ptr->GetGeometryType() ==
            Geometry::GeometryType::Unspecified) {
        return false;
    } else if (geometry_ptr->GetGeometryType() ==
            Geometry::GeometryType::PointCloud) {
        auto renderer_ptr = std::make_shared<glsl::PointCloudRenderer>();
        if (renderer_ptr->AddGeometry(geometry_ptr) == false) {
            return false;
        }
        geometry_renderer_ptrs_.push_back(renderer_ptr);
    } else if (geometry_ptr->GetGeometryType() ==
            Geometry::GeometryType::LineSet) {
        auto renderer_ptr = std::make_shared<glsl::LineSetRenderer>();
        if (renderer_ptr->AddGeometry(geometry_ptr) == false) {
            return false;
        }
        geometry_renderer_ptrs_.push_back(renderer_ptr);
    } else if (geometry_ptr->GetGeometryType() ==
            Geometry::GeometryType::TriangleMesh) {
        auto renderer_ptr = std::make_shared<glsl::TriangleMeshRenderer>();
        if (renderer_ptr->AddGeometry(geometry_ptr) == false) {
            return false;
        }
        geometry_renderer_ptrs_.push_back(renderer_ptr);
    } else if (geometry_ptr->GetGeometryType() ==
            Geometry::GeometryType::Image) {
        auto renderer_ptr = std::make_shared<glsl::ImageRenderer>();
        if (renderer_ptr->AddGeometry(geometry_ptr) == false) {
            return false;
        }
        geometry_renderer_ptrs_.push_back(renderer_ptr);
    } else {
        return false;
    }

    geometry_ptrs_.push_back(geometry_ptr);
    view_control_ptr_->FitInGeometry(*geometry_ptr);
    ResetViewPoint();
    PrintDebug("Add geometry and update bounding box to %s\n",
            view_control_ptr_->GetBoundingBox().GetPrintInfo().c_str());
    return UpdateGeometry();
}

bool Visualizer::AddUbitrackImage(std::shared_ptr<const UbitrackImage> geometry_ptr)
{
    if (is_initialized_ == false) {
        return false;
    }
    glfwMakeContextCurrent(window_);

    if (geometry_ptr->GetGeometryType() ==
        Geometry::GeometryType::Image) {
        auto renderer_ptr = std::make_shared<glsl::UbitrackImageRenderer>();
        if (renderer_ptr->AddGeometry(geometry_ptr) == false) {
            return false;
        }
        geometry_renderer_ptrs_.push_back(renderer_ptr);
    } else {
        return false;
    }

    geometry_ptrs_.push_back(geometry_ptr);
    view_control_ptr_->FitInGeometry(*geometry_ptr);
    ResetViewPoint();
    PrintDebug("Add geometry and update bounding box to %s\n",
            view_control_ptr_->GetBoundingBox().GetPrintInfo().c_str());
    return UpdateGeometry();
}


bool Visualizer::UpdateGeometry()
{
    glfwMakeContextCurrent(window_);
    bool success = true;
    for (const auto &renderer_ptr : geometry_renderer_ptrs_) {
        success = (success && renderer_ptr->UpdateGeometry());
    }
    UpdateRender();
    return success;
}

void Visualizer::UpdateRender()
{
    is_redraw_required_ = true;
}

bool Visualizer::HasGeometry() const
{
    return !geometry_ptrs_.empty();
}

void Visualizer::PrintVisualizerHelp()
{
    PrintInfo("  -- Mouse view control --\n");
    PrintInfo("    Left button + drag        : Rotate.\n");
    PrintInfo("    Ctrl + left button + drag : Translate.\n");
    PrintInfo("    Wheel                     : Zoom in/out.\n");
    PrintInfo("\n");
    PrintInfo("  -- Keyboard view control --\n");
    PrintInfo("    [/]          : Increase/decrease field of view.\n");
    PrintInfo("    R            : Reset view point.\n");
    PrintInfo("    Ctrl/Cmd + C : Copy current view status into the clipboard.\n");
    PrintInfo("    Ctrl/Cmd + V : Paste view status from clipboard.\n");
    PrintInfo("\n");
    PrintInfo("  -- General control --\n");
    PrintInfo("    Q, Esc       : Exit window.\n");
    PrintInfo("    H            : Print help message.\n");
    PrintInfo("    P, PrtScn    : Take a screen capture.\n");
    PrintInfo("    D            : Take a depth capture.\n");
    PrintInfo("    O            : Take a capture of current rendering settings.\n");
    PrintInfo("\n");
    PrintInfo("  -- Render mode control --\n");
    PrintInfo("    L            : Turn on/off lighting.\n");
    PrintInfo("    +/-          : Increase/decrease point size.\n");
    PrintInfo("    N            : Turn on/off point cloud normal rendering.\n");
    PrintInfo("    S            : Toggle between mesh flat shading and smooth shading.\n");
    PrintInfo("    W            : Turn on/off mesh wireframe.\n");
    PrintInfo("    B            : Turn on/off back face rendering.\n");
    PrintInfo("    I            : Turn on/off image zoom in interpolation.\n");
    PrintInfo("    T            : Toggle among image render:\n");
    PrintInfo("                   no stretch / keep ratio / freely stretch.\n");
    PrintInfo("\n");
    PrintInfo("  -- Color control --\n");
    PrintInfo("    0..4,9       : Set point cloud color option.\n");
    PrintInfo("                   0 - Default behavior, render point color.\n");
    PrintInfo("                   1 - Render point color.\n");
    PrintInfo("                   2 - x coordinate as color.\n");
    PrintInfo("                   3 - y coordinate as color.\n");
    PrintInfo("                   4 - z coordinate as color.\n");
    PrintInfo("                   9 - normal as color.\n");
    PrintInfo("    Ctrl + 0..4,9: Set mesh color option.\n");
    PrintInfo("                   0 - Default behavior, render uniform gray color.\n");
    PrintInfo("                   1 - Render point color.\n");
    PrintInfo("                   2 - x coordinate as color.\n");
    PrintInfo("                   3 - y coordinate as color.\n");
    PrintInfo("                   4 - z coordinate as color.\n");
    PrintInfo("                   9 - normal as color.\n");
    PrintInfo("    Shift + 0..4 : Color map options.\n");
    PrintInfo("                   0 - Gray scale color.\n");
    PrintInfo("                   1 - JET color map.\n");
    PrintInfo("                   2 - SUMMER color map.\n");
    PrintInfo("                   3 - WINTER color map.\n");
    PrintInfo("                   4 - HOT color map.\n");
    PrintInfo("\n");
}

}    // namespace artekmed
