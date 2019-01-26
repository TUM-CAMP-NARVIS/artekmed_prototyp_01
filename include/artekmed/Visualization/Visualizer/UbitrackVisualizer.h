//
// Created by Ulrich Eck on 15.03.18.
//

#ifndef ARTEKMED_UBITRACKVISUALIZER_H
#define ARTEKMED_UBITRACKVISUALIZER_H

#include <GL/glew.h>

#include <artekmed/Visualization/Visualizer/Visualizer.h>

#include "artekmed/Visualization/Utility/UbitrackImage.h"

namespace artekmed {

    using namespace open3d;

class UbitrackVisualizer : public Visualizer
{
public:
    UbitrackVisualizer();
    ~UbitrackVisualizer() override;
    UbitrackVisualizer(const UbitrackVisualizer &) = delete;
    UbitrackVisualizer &operator=(const UbitrackVisualizer &) = delete;

public:
    void PrintVisualizerHelp() override;

    bool AddUbitrackImage(std::shared_ptr<const UbitrackImage> geometry_ptr);

protected:
    void UpdateWindowTitle() override;
    void Render() override;

    virtual bool StartUbitrack() = 0;
    virtual bool StopUbitrack() = 0;

    bool InitOpenGL() override;
    bool InitViewControl() override;
    bool InitRenderOption() override;

    void WindowCloseCallback(GLFWwindow *window) override;

    void MouseMoveCallback(GLFWwindow* window, double x, double y) override;
    void MouseScrollCallback(GLFWwindow* window, double x, double y) override;
    void MouseButtonCallback(GLFWwindow* window,
            int button, int action, int mods) override;
    void KeyPressCallback(GLFWwindow *window,
            int key, int scancode, int action, int mods) override;

};

}	// namespace open3d

#endif //ARTEKMED_UBITRACKVISUALIZER_H
