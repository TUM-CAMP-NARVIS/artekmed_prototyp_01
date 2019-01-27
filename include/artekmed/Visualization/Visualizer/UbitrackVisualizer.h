//
// Created by Ulrich Eck on 15.03.18.
//

#ifndef ARTEKMED_UBITRACKVISUALIZER_H
#define ARTEKMED_UBITRACKVISUALIZER_H

#include <utVision/OpenGLPlatform.h>

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

protected:
    void UpdateWindowTitle() override;

    bool InitOpenGL() override;
    bool InitViewControl() override;

    void WindowCloseCallback(GLFWwindow *window) override;

};

}	// namespace open3d

#endif //ARTEKMED_UBITRACKVISUALIZER_H
