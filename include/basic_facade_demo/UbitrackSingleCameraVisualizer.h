//
// Created by Ulrich Eck on 15.03.18.
//

#ifndef BASIC_FACADE_DEMO_UBITRACKSINGLECAMERAVISUALIZER_H
#define BASIC_FACADE_DEMO_UBITRACKSINGLECAMERAVISUALIZER_H

#include "basic_facade_demo/UbitrackVisualizer.h"
#include "basic_facade_demo/UbitrackSingleCameraConnector.h"

namespace three {

class UbitrackSingleCameraVisualizer : public UbitrackVisualizer
{
public:
    UbitrackSingleCameraVisualizer();
    ~UbitrackSingleCameraVisualizer() override;
    UbitrackSingleCameraVisualizer(const UbitrackSingleCameraVisualizer &) =
    delete;
    UbitrackSingleCameraVisualizer &operator=(
            const UbitrackSingleCameraVisualizer &) = delete;

public:
    void SetUbitrackConnector(std::shared_ptr<UbitrackSingleCameraConnector>& connector) throw();

protected:

    std::shared_ptr<Ubitrack::Facade::BasicImageMeasurement > ubitrack_camera_image_ptr;
    std::unique_ptr<Ubitrack::Facade::BasicTextureUpdate> ubitrack_textureupdate_left_ptr;
    std::shared_ptr<UbitrackSingleCameraConnector> ubitrack_connector_ptr;
};

}	// namespace three



#endif //BASIC_FACADE_DEMO_UBITRACKSINGLECAMERAVISUALIZER_H
