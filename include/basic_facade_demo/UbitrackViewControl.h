//
// Created by Ulrich Eck on 15.03.18.
//

#ifndef BASIC_FACADE_DEMO_UBITRACKVIEWCONTROL_H
#define BASIC_FACADE_DEMO_UBITRACKVIEWCONTROL_H

#include <Visualization/Visualizer/ViewControl.h>

namespace three {

class UbitrackViewControl : public ViewControl
{

public:
    UbitrackViewControl()
            : camera_intrinsics_(Eigen::Matrix3d::Identity())
            , camera_resolution_(Eigen::Vector2i::Identity())
            , camera_intrinsics_available_(false) {}

    void Reset() override;

    void SetUbitrackViewMatrices();

    double GetNear();
    double GetFar();

    void SetCameraModel(const Eigen::Matrix4d& projection_, const Eigen::Matrix3d& intrinsics_, const Eigen::Vector2i& resolution_);
    void SetCameraExtrinsics(const Eigen::Matrix4d& view_matrix);

    // these need to be disabled
    void ChangeFieldOfView(double step) override;
    void Scale(double scale) override;
    void Rotate(double x, double y, double xo, double yo) override;
    void Translate(double x, double y, double xo, double yo) override;

    std::string GetStatusString() const;

    void PrintDebugMatrices();


protected:

    Eigen::Matrix4d ComputeProjectionMatrix(const Eigen::Matrix3d& intrinsics, const Eigen::Vector2i& resolution, double n, double f);

protected:
    Eigen::Matrix3d camera_intrinsics_;
    Eigen::Vector2i camera_resolution_;
    bool camera_intrinsics_available_;

};

}	// namespace three


#endif //BASIC_FACADE_DEMO_UBITRACKVIEWCONTROL_H
