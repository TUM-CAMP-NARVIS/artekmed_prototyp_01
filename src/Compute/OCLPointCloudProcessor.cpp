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

    float fx = intr_rect_ir(0,0);
    float ppx = intr_rect_ir(0,2);
    float fy = intr_rect_ir(1,1);
    float ppy = intr_rect_ir(1,2);

    cv::UMat umat_src_depth = depth_img_rect.getUMat(cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
    cv::UMat umat_dst_depth(depth_img_rect.size(), CV_32FC3, cv::ACCESS_WRITE, cv::USAGE_ALLOCATE_DEVICE_MEMORY);

    cv::ocl::Kernel kernel;
    if (!GetKernel("depthmap_to_points", kernel)) {
        PrintProgramWarning("no kernel .. skip");
        return;
    }

    kernel.args(
            cv::ocl::KernelArg::ReadOnlyNoSize(umat_src_depth),
            cv::ocl::KernelArg::ReadWrite(umat_dst_depth)
    );

    cv::Size dims = depth_img_rect.size();
    size_t globalThreads[3] = { (size_t)dims.height, (size_t)dims.width, 1 };
    size_t localThreads[3] = { 16, 16, 1 };
    bool success = kernel.run(3, globalThreads, localThreads, true);
    if (!success){
        cout << "Failed running the kernel.. " << endl;
        return;
    }

    // Download the dst data from the device (?)
    cv::Mat mat_dst = umat_dst_depth.getMat(cv::ACCESS_READ);


}
