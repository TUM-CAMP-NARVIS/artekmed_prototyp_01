//
// Created by netlabs on 28.01.19.
//

#include "artekmed/Compute/OCLPointCloudProcessing.h"


#include <iostream>
#include <fstream>
#include <string>
#include <iterator>
#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>

#include "artekmed/Compute/OCLKernels.h"

using namespace std;

void OCLPointCloudProcessing::test_oclkernel()
{
    if (!cv::ocl::haveOpenCL())
    {
        cout << "OpenCL is not avaiable..." << endl;
        return;
    }
    cv::ocl::Context context;
    if (!context.create(cv::ocl::Device::TYPE_GPU))
    {
        cout << "Failed creating the context..." << endl;
        return;
    }

    // In OpenCV 3.0.0 beta, only a single device is detected.
    cout << context.ndevices() << " GPU devices are detected." << endl;
    for (int i = 0; i < context.ndevices(); i++)
    {
        cv::ocl::Device device = context.device(i);
        cout << "name                 : " << device.name() << endl;
        cout << "available            : " << device.available() << endl;
        cout << "imageSupport         : " << device.imageSupport() << endl;
        cout << "OpenCL_C_Version     : " << device.OpenCL_C_Version() << endl;
        cout << endl;
    }

    // Select the first device
    cv::ocl::Device(context.device(0));

    // Transfer Mat data to the device
//    cv::Mat mat_src = cv::imread("Lena.png", cv::IMREAD_GRAYSCALE);
//    mat_src.convertTo(mat_src, CV_32F, 1.0 / 255);
//    cv::UMat umat_src = mat_src.getUMat(cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
//    cv::UMat umat_dst(mat_src.size(), CV_32F, cv::ACCESS_WRITE, cv::USAGE_ALLOCATE_DEVICE_MEMORY);

    size_t data_size = 1024;

    cv::Mat mat_src(cv::Size(data_size, 1), CV_32F);
    double mean = 0.0;
    double stddev = 500.0 / 3.0; // 99.7% of values will be inside [-500, +500] interval
    cv::randn(mat_src, cv::Scalar(mean), cv::Scalar(stddev));

    cv::UMat umat_src = mat_src.getUMat(cv::ACCESS_READ, cv::USAGE_ALLOCATE_DEVICE_MEMORY);
    cv::UMat umat_dst(data_size, CV_32F, cv::ACCESS_WRITE, cv::USAGE_ALLOCATE_DEVICE_MEMORY);


    cv::ocl::ProgramSource programSource(open3d::glsl::HelloWorld);

    // Compile the kernel code
    cv::String errmsg;
    cv::String buildopt = cv::format("-D dstT=%s", cv::ocl::typeToStr(umat_dst.depth())); // "-D dstT=float"
    cv::ocl::Program program = context.getProg(programSource, buildopt, errmsg);

//    cv::ocl::Image2D image(umat_src);
//    float shift_x = 100.5;
//    float shift_y = -50.0;


    cv::ocl::Kernel kernel("square", program);

    kernel.args(
            cv::ocl::KernelArg::ReadOnlyNoSize(umat_src),
            cv::ocl::KernelArg::ReadWrite(umat_dst)
            );

    size_t globalThreads[3] = { data_size, 1, 1 };
    //size_t localThreads[3] = { 16, 16, 1 };
    bool success = kernel.run(3, globalThreads, NULL, true);
    if (!success){
        cout << "Failed running the kernel..." << endl;
        return;
    }

    // Download the dst data from the device (?)
    cv::Mat mat_dst = umat_dst.getMat(cv::ACCESS_READ);

//    cv::imshow("src", mat_src);
//    cv::imshow("dst", mat_dst);
//    cv::waitKey();
}