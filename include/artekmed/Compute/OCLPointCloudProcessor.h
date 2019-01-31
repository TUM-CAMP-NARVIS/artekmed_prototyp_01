//
// Created by netlabs on 28.01.19.
//

#ifndef ARTEKMED_P1_OCLPOINTCLOUDPROCESSING_H
#define ARTEKMED_P1_OCLPOINTCLOUDPROCESSING_H

#include "artekmed/Compute/ProgramWrapper.h"

#include <Core/Geometry/PointCloud.h>


namespace artekmed {


    using namespace open3d;

    namespace compute {

        class OCLPointCloudProcessor : public ProgramWrapper {

        public:
            ~OCLPointCloudProcessor() override { Release(); }
        protected:
            bool Compile() final;
            void Release() final;

            std::string GenerateBuildOptions() override {
                std::ostringstream oss;
                oss.precision(16);
                oss << std::scientific;

                oss << " -D DATA_TYPE=float";
                oss << " -D INPUT_TYPE=ushort";
                oss << " -D OUTPUT_TYPE=float3";

                const cv::ocl::Device & dev = cv::ocl::Device::getDefault();
                int pxPerWIy = (dev.isIntel() && (dev.type() & cv::ocl::Device::TYPE_GPU)) ? 4 : 1;
                oss << " -D PIX_PER_WI_Y=%d" << pxPerWIy;


//                // Rotation
//                oss << " -D r00=" << rotation.at<double>(0, 0) << "f";
//                oss << " -D r01=" << rotation.at<double>(0, 1) << "f";
//                oss << " -D r02=" << rotation.at<double>(0, 2) << "f";
//                oss << " -D r10=" << rotation.at<double>(1, 0) << "f";
//                oss << " -D r11=" << rotation.at<double>(1, 1) << "f";
//                oss << " -D r12=" << rotation.at<double>(1, 2) << "f";
//                oss << " -D r20=" << rotation.at<double>(2, 0) << "f";
//                oss << " -D r21=" << rotation.at<double>(2, 1) << "f";
//                oss << " -D r22=" << rotation.at<double>(2, 2) << "f";
//
//                // Translation
//                oss << " -D tx=" << translation.at<double>(0, 0) << "f";
//                oss << " -D ty=" << translation.at<double>(1, 0) << "f";
//                oss << " -D tz=" << translation.at<double>(2, 0) << "f";
//
//                // Camera parameter upscaled depth
//                oss << " -D fxR=" << cameraMatrixRegistered.at<double>(0, 0) << "f";
//                oss << " -D fyR=" << cameraMatrixRegistered.at<double>(1, 1) << "f";
//                oss << " -D cxR=" << cameraMatrixRegistered.at<double>(0, 2) << "f";
//                oss << " -D cyR=" << cameraMatrixRegistered.at<double>(1, 2) << "f";
//                oss << " -D fxRInv=" << (1.0 / cameraMatrixRegistered.at<double>(0, 0)) << "f";
//                oss << " -D fyRInv=" << (1.0 / cameraMatrixRegistered.at<double>(1, 1)) << "f";
//
//                // Clipping distances
//                oss << " -D zNear=" << (uint16_t)(zNear * 1000);
//                oss << " -D zFar=" << (uint16_t)(zFar * 1000);
//
//                // Size registered image
//                oss << " -D heightR=" << sizeRegistered.height;
//                oss << " -D widthR=" << sizeRegistered.width;
//
//                // Size depth image
//                oss << " -D heightD=" << sizeDepth.height;
//                oss << " -D widthD=" << sizeDepth.width;

                return oss.str();
            }


        public:
            OCLPointCloudProcessor(const std::string& name)
                    : ProgramWrapper(name) {

            };

            void buildPointCloud(
                    const cv::Mat& depth_img_rect,
                    const Eigen::Matrix3f& intr_rect_ir,
                    open3d::PointCloud& cloud,
                    float depth_scale_factor=1.0);



//            void buildColoredPointCloud(
//                    const cv::Mat& depth_img_rect,
//                    const cv::Mat& color_img_rect,
//                    const Eigen::Matrix3f& intr_rect_ir,
//                    open3d::PointCloud& cloud,
//                    float depth_scale_factor=1.0);



        };

    }
}
#endif //ARTEKMED_P1_OCLPOINTCLOUDPROCESSING_H
