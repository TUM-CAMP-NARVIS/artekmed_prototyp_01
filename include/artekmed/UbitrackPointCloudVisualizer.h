//
// Created by Ulrich Eck on 15.03.18.
//

#ifndef ARTEKMED_UBITRACKSINGLECAMERAVISUALIZER_H
#define ARTEKMED_UBITRACKSINGLECAMERAVISUALIZER_H

#include "artekmed/Visualization/Visualizer/UbitrackVisualizer.h"
#include "artekmed/Visualization/Utility/UbitrackImage.h"
#include "artekmed/UbitrackPointCloudConnector.h"

#include "artekmed/Compute/OCLPointCloudProcessor.h"
#include "artekmed/Compute/OCLTestProcessor.h"

namespace artekmed {

    using namespace open3d;

class UbitrackPointCloudVisualizer : public UbitrackVisualizer
{
public:
    UbitrackPointCloudVisualizer();
    ~UbitrackPointCloudVisualizer() override;
    UbitrackPointCloudVisualizer(const UbitrackPointCloudVisualizer &) = delete;
    UbitrackPointCloudVisualizer &operator=(const UbitrackPointCloudVisualizer &) = delete;

public:
    void SetUbitrackConnector(std::shared_ptr<UbitrackPointCloudConnector>& connector) throw();
    void addPointCloud(std::shared_ptr<open3d::PointCloud>& point_cloud);

protected:

    bool StartUbitrack() override;
    bool StopUbitrack() override;

    std::vector<std::shared_ptr<open3d::PointCloud>> ubitrack_pointclouds_ptr;


    std::shared_ptr<UbitrackPointCloudConnector> ubitrack_connector_ptr;
    bool ocl_initialized = false;

};

}	// namespace open3d



#endif //ARTEKMED_UBITRACKSINGLECAMERAVISUALIZER_H
