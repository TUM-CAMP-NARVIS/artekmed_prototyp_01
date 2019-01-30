//
// Created by netlabs on 28.01.19.
//

#include "artekmed/Compute/OCLPointCloudProcessor.h"


#include <iostream>
#include <fstream>
#include <string>
#include <iterator>
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>

#include "artekmed/Compute/OCLKernels.h"

using namespace std;
using namespace artekmed::compute;


bool OCLPointCloudProcessor::Compile()
{
    using namespace open3d::glsl;

    if (CompileProgram(PointCloudKernels) == false) {
        PrintProgramWarning("Compiling program failed.");
        return false;
    }
    return true;
}

void OCLPointCloudProcessor::Release()
{
    ReleaseProgram();
}

void OCLPointCloudProcessor::buildPointCloud(const cv::Mat &depth_img_rect, const Eigen::Matrix3f &intr_rect_ir,
                                             open3d::PointCloud &cloud, float depth_scale_factor) {

//    size_t data_size = 1024;
//
//    cv::Mat mat_src(cv::Size(data_size, 1), CV_32F);
//    double mean = 0.0;
//    double stddev = 500.0 / 3.0; // 99.7% of values will be inside [-500, +500] interval
//    cv::randn(mat_src, cv::Scalar(mean), cv::Scalar(stddev));
//
//    cv::UMat umat_src = mat_src.getUMat(cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
//    cv::UMat umat_dst(data_size, CV_32F, cv::ACCESS_WRITE, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
//
//
//
//    auto& kernel = GetKernel("depthmap_to_points");
//
//
//    kernel.args(
//            cv::ocl::KernelArg::ReadOnlyNoSize(umat_src),
//            cv::ocl::KernelArg::ReadWrite(umat_dst)
//    );
//
//    size_t globalThreads[3] = { data_size, 1, 1 };
//    //size_t localThreads[3] = { 16, 16, 1 };
//    bool success = kernel.run(3, globalThreads, NULL, true);
//    if (!success){
//        cout << "Failed running the kernel..." << endl;
//        return;
//    }
//
//    // Download the dst data from the device (?)
//    cv::Mat mat_dst = umat_dst.getMat(cv::ACCESS_READ);
//

}
