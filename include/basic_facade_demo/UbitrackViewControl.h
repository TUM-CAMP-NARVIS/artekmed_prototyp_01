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
    void Reset() override;

    // this overrides a non-virtual function - bad design, maybe add pull request to Open3D to fix it.
    void SetViewMatrices(
            const Eigen::Matrix4d &model_matrix = Eigen::Matrix4d::Identity());

    void SetCameraIntrinsics(const Eigen::Matrix3d& intrinsics_, const Eigen::Vector2i&  resolution_);
    void SetCameraExtrinsics(const Eigen::Matrix4d& view_matrix);

    // these need to be disabled
    void ChangeFieldOfView(double step) override;
    void Scale(double scale) override;
    void Rotate(double x, double y, double xo, double yo) override;
    void Translate(double x, double y, double xo, double yo) override;

    std::string GetStatusString() const;


protected:

    Eigen::Matrix4d ComputeProjectionMatrix(const Eigen::Matrix3d& intrinsics, const Eigen::Vector2i& resolution, double n, double f);

protected:
    Eigen::Matrix3d camera_intrinsics_;
    Eigen::Vector2i camera_resolution_;


};

}	// namespace three


#endif //BASIC_FACADE_DEMO_UBITRACKVIEWCONTROL_H
