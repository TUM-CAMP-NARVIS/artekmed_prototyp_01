//
// Created by Ulrich Eck on 15.03.18.
//

#ifndef BASIC_FACADE_DEMO_UBITRACKVISUALIZER_H
#define BASIC_FACADE_DEMO_UBITRACKVISUALIZER_H

#include <Visualization/Visualizer/Visualizer.h>

#include <utFacade/BasicTextureUpdate.h>
#include "basic_facade_demo/UbitrackBaseConnector.h"

namespace three {

class UbitrackVisualizer : public Visualizer
{
public:

public:
    UbitrackVisualizer();
    ~UbitrackVisualizer() override;
    UbitrackVisualizer(const UbitrackVisualizer &) =
    delete;
    UbitrackVisualizer &operator=(
            const UbitrackVisualizer &) = delete;

public:
    void PrintVisualizerHelp() override;

protected:
    void UpdateWindowTitle() override;

    bool InitViewControl() override;
    void MouseMoveCallback(GLFWwindow* window, double x, double y) override;
    void MouseScrollCallback(GLFWwindow* window, double x, double y) override;
    void MouseButtonCallback(GLFWwindow* window,
            int button, int action, int mods) override;
    void KeyPressCallback(GLFWwindow *window,
            int key, int scancode, int action, int mods) override;

};

}	// namespace three



#endif //BASIC_FACADE_DEMO_UBITRACKVISUALIZER_H
