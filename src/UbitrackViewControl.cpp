//
// Created by Ulrich Eck on 15.03.18.
//

#include <iostream>
#include <cmath>
#include "basic_facade_demo/UbitrackViewControl.h"

#include <IO/ClassIO/IJsonConvertibleIO.h>

namespace three{

void UbitrackViewControl::SetCameraIntrinsics(const Eigen::Matrix3d& intrinsics_, const Eigen::Vector2i&  resolution_)
{
    std::cout << "intrinsics: " << intrinsics_ << std::endl;
    std::cout << "resolution: " << resolution_ << std::endl;
    camera_intrinsics_ = intrinsics_;
    camera_resolution_ = resolution_;
}

void UbitrackViewControl::SetCameraExtrinsics(const Eigen::Matrix4d& view_matrix) {
    std::cout << "extrinsics: " << view_matrix << std::endl;
    view_matrix_ = view_matrix.cast<GLfloat>();
}

// this function hides a non-virtual function - bad design!!
void UbitrackViewControl::SetViewMatrices(
        const Eigen::Matrix4d &model_matrix/* = Eigen::Matrix4d::Identity()*/)
{
    if (window_height_ <= 0 || window_width_ <= 0) {
        PrintWarning("[ViewControl] SetViewPoint() failed because window height and width are not set.");
        return;
    }
    glViewport(0, 0, window_width_, window_height_);

    // Perspective projection
    z_near_ = std::max(0.01 * bounding_box_.GetSize(),
            distance_ - 3.0 * bounding_box_.GetSize());
    z_far_ = distance_ + 3.0 * bounding_box_.GetSize();

    projection_matrix_ = ComputeProjectionMatrix(camera_intrinsics_, camera_resolution_, z_near_, z_far_).cast<GLfloat>();

    std::cout << "projection matrix: " << projection_matrix_ << std::endl;

    model_matrix_ = model_matrix.cast<GLfloat>();
    MVP_matrix_ = projection_matrix_ * view_matrix_ * model_matrix_;

}

void UbitrackViewControl::Reset()
{
    ViewControl::Reset();
    // get intrinsics and resolution from ubitrack here
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