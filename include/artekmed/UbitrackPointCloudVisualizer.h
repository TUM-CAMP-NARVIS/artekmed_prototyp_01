//
// Created by Ulrich Eck on 15.03.18.
//

#ifndef ARTEKMED_UBITRACKSINGLECAMERAVISUALIZER_H
#define ARTEKMED_UBITRACKSINGLECAMERAVISUALIZER_H

#include "artekmed/UbitrackVisualizer.h"
#include "artekmed/UbitrackImage.h"
#include "artekmed/UbitrackPointCloudConnector.h"

namespace open3d {

class UbitrackPointCloudVisualizer : public UbitrackVisualizer
{
public:
    UbitrackPointCloudVisualizer();
    ~UbitrackPointCloudVisualizer() override;
    UbitrackPointCloudVisualizer(const UbitrackPointCloudVisualizer &) = delete;
    UbitrackPointCloudVisualizer &operator=(const UbitrackPointCloudVisualizer &) = delete;

public:
    void SetUbitrackConnector(std::shared_ptr<UbitrackPointCloudConnector>& connector) throw();
    void setPointCloud1(std::shared_ptr<open3d::PointCloud>& point_cloud);
    void setPointCloud2(std::shared_ptr<open3d::PointCloud>& point_cloud);
    void setPointCloud3(std::shared_ptr<open3d::PointCloud>& point_cloud);

protected:

    bool StartUbitrack() override;
    bool StopUbitrack() override;

    std::shared_ptr<open3d::PointCloud> ubitrack_camera01_pointcloud_ptr;
    std::shared_ptr<open3d::PointCloud> ubitrack_camera02_pointcloud_ptr;
    std::shared_ptr<open3d::PointCloud> ubitrack_camera03_pointcloud_ptr;

    std::shared_ptr<open3d::Geometry3D> ubitrack_camera02_pose_ptr;
    std::shared_ptr<open3d::Geometry3D> ubitrack_camera03_pose_ptr;

    std::shared_ptr<UbitrackPointCloudConnector> ubitrack_connector_ptr;
};

}	// namespace open3d



#endif //ARTEKMED_UBITRACKSINGLECAMERAVISUALIZER_H
