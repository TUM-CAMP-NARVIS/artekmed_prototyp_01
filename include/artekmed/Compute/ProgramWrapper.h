//
// Created by netlabs on 29.01.19.
//

#pragma once

#include <utVision/OpenGLPlatform.h>
#include <utVision/OpenCLManager.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>


#include <Core/Geometry/Geometry.h>

namespace artekmed {


    using namespace open3d;

    namespace compute {

        class ProgramWrapper
        {
        public:
            virtual ~ProgramWrapper() {}
            ProgramWrapper(const ProgramWrapper &) = delete;
            ProgramWrapper &operator=(const ProgramWrapper &) = delete;

        protected:
            ProgramWrapper(const std::string &name) : program_name_(name) {}

        public:

            cv::ocl::Kernel GetKernel(const char * kernel_name) {
                return cv::ocl::Kernel(kernel_name, m_program);
            }

            const std::string &GetProgramName() const { return program_name_; }

            void PrintProgramWarning(const std::string &message) const;

        protected:
            /// Function to compile shader
            /// In a derived class, this must be declared as final, and called from
            /// the constructor.
            virtual bool Compile() = 0;

            /// Function to release resource
            /// In a derived class, this must be declared as final, and called from
            /// the destructor.
            virtual void Release() = 0;

            virtual std::string GenerateBuildOptions() = 0;

        protected:
            bool CompileProgram(const char *const kernel_code);
            void ReleaseProgram();


        protected:
            cv::ocl::Program m_program;

            bool compiled_ = false;
            bool bound_ = false;

            void SetProgramName(const std::string &kernel_name) {
                program_name_ = kernel_name;
            }

        private:
            std::string program_name_ = "ProgramWrapper";
        };

    }    // namespace open3d::glsl

}    // namespace open3d