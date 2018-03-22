//
// Created by Ulrich Eck on 15.03.18.
//

#include <strstream>
#include <cmath>
#include <Eigen/Dense>

#include "basic_facade_demo/UbitrackViewControl.h"

#include <IO/ClassIO/IJsonConvertibleIO.h>

namespace three{

void UbitrackViewControl::SetCameraModel(const Eigen::Matrix4d& projection_, const Eigen::Matrix3d& intrinsics_, const Eigen::Vector2i& resolution_)
{
    camera_intrinsics_ = intrinsics_;
    camera_resolution_ = resolution_;
//    projection_matrix_ = projection_.cast<GLfloat>();
    camera_intrinsics_available_ = true;
}

void UbitrackViewControl::SetCameraExtrinsics(const Eigen::Matrix4d& view_matrix) {
    view_matrix_ = view_matrix.cast<GLfloat>();
}

double UbitrackViewControl::GetNear()
{
    return 0.001;
//            std::max(0.01 * bounding_box_.GetSize(),
//            distance_ - 3.0 * bounding_box_.GetSize());
}

double UbitrackViewControl::GetFar()
{
    return 1000.0;
//    distance_ + 3.0 * bounding_box_.GetSize();
}



void UbitrackViewControl::SetUbitrackViewMatrices()
{
    if (!camera_intrinsics_available_) {
        ViewControl::SetViewMatrices();
        return;
    }

    if (window_height_ <= 0 || window_width_ <= 0) {
        PrintWarning("[ViewControl] SetViewPoint() failed because window height and width are not set.");
        return;
    }
    glViewport(0, 0, window_width_, window_height_);

    // we track the camera therefore model_matrix is always identity
    Eigen::Matrix4d model_matrix = Eigen::Matrix4d::Identity();

//    z_near_ = GetNear();
//    z_far_ = GetFar();
//
//    projection_matrix_ = ComputeProjectionMatrix(camera_intrinsics_, camera_resolution_, z_near_, z_far_).cast<GLfloat>();

    model_matrix_ = model_matrix.cast<GLfloat>();
    MVP_matrix_ = projection_matrix_ * view_matrix_ * model_matrix_;

}

void UbitrackViewControl::Reset()
{
    ViewControl::Reset();
    camera_intrinsics_ = Eigen::Matrix3d::Identity();
    camera_resolution_ = Eigen::Vector2i::Identity();
    camera_intrinsics_available_ = false;

}

void UbitrackViewControl::ChangeFieldOfView(double step)
{
    // camera intrinsics are configured
}

void UbitrackViewControl::Scale(double scale)
{
    // camera is controlled by tracker
}

void UbitrackViewControl::Rotate(double x, double y, double xo,
        double yo)
{
    // camera is controlled by tracker
}

void UbitrackViewControl::Translate(double x, double y, double xo,
        double yo)
{
    // camera is controlled by tracker
}


std::string UbitrackViewControl::GetStatusString() const
{
    std::string prefix;

    char buffer[DEFAULT_IO_BUFFER_SIZE];
    sprintf(buffer, "idle");
    return prefix + std::string(buffer);
}

void UbitrackViewControl::PrintDebugMatrices() {
    std::ostrstream s;
    s << "============ Matrix Debug ============\n";
    s << "Intrinsics Matrix: \n";
    s << camera_intrinsics_ << "\n";
    s << "Extrinsics Matrix: \n";
    s << view_matrix_ << "\n";
    s << "Projection Matrix: \n";
    s << projection_matrix_ << "\n";
    s << "MVP Matrix: \n";
    s << MVP_matrix_<< "\n";
    PrintDebug(s.str());
}

Eigen::Matrix4d UbitrackViewControl::ComputeProjectionMatrix(
        const Eigen::Matrix3d& intrinsics, const Eigen::Vector2i& resolution,
        double n, double f) {

    double l = 0;
    double r = (double)resolution(0);
    double b = 0;
    double t = (double)resolution(1);

    Eigen::Matrix4d m2;
    m2.setZero();
    m2.topLeftCorner<3,3>() = intrinsics;

    double norm = std::sqrt( m2( 2, 0 )*m2( 2, 0 ) + m2( 2, 1 )*m2( 2, 1 ) + m2( 2, 2 )*m2( 2, 2 ) );
    double nf = ( -f - n );
    double add = f * n * norm;

    // copy 3rd row to 4th row
    m2.row(3) = m2.row(2);

    // factor for normalization
    m2.row(2) *= nf;

    m2(2,3) += add;

    Eigen::Matrix4d ortho;
    ortho.setIdentity();

    ortho( 0, 0 ) = static_cast< double >( 2.0 ) / ( r - l );
    ortho( 0, 1 ) = static_cast< double >( 0.0 );
    ortho( 0, 2 ) = static_cast< double >( 0.0 );
    ortho( 0, 3 ) = ( r + l ) / ( l - r );
    ortho( 1, 0 ) = static_cast< double >( 0.0 );
    ortho( 1, 1 ) = static_cast< double >( 2.0 ) / ( t - b );
    ortho( 1, 2 ) = static_cast< double >( 0.0 );
    ortho( 1, 3 ) = ( t + b ) / ( b - t );
    ortho( 2, 0 ) = static_cast< double >( 0.0 );
    ortho( 2, 1 ) = static_cast< double >( 0.0 );
    ortho( 2, 2 ) = static_cast< double >( 2.0 ) / ( n - f );
    ortho( 2, 3 ) = ( f + n ) / ( n - f );
    ortho( 3, 0 ) = static_cast< double >( 0.0 );
    ortho( 3, 1 ) = static_cast< double >( 0.0 );
    ortho( 3, 2 ) = static_cast< double >( 0.0 );
    ortho( 3, 3 ) = static_cast< double >( 1.0 );

    return ortho * m2;
}

}	// namespace three