//
// Created by Ulrich Eck on 15.03.18.
//

#ifndef ARTEKMED_UBITRACKSINGLECAMERACONNECTOR_H
#define ARTEKMED_UBITRACKSINGLECAMERACONNECTOR_H

#include <Eigen/Core>
#include <Core/Geometry/PointCloud.h>
#include "artekmed/Ubitrack/BaseConnector.h"

#include <boost/shared_ptr.hpp>
#include <utVision/Image.h>
#include "utComponents/ApplicationPullSink.h"
#include "utComponents/ApplicationPushSink.h"
#include "utComponents/ApplicationPullSource.h"
#include "utComponents/ApplicationPushSource.h"
#include "utComponents/ApplicationEndpointsVision.h"

class UbitrackPointCloudConnector: public UbitrackBaseConnector {
public:
    explicit UbitrackPointCloudConnector(const std::string& _components_path);
    virtual ~UbitrackPointCloudConnector() = default;


    // configure cameras
    void add_cameras() override;

};

#endif //ARTEKMED_UBITRACKSINGLECAMERACONNECTOR_H
