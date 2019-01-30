//
// Created by netlabs on 29.01.19.
//

#include "artekmed/Compute/ProgramWrapper.h"

#include <Core/Geometry/Geometry.h>
#include <Core/Utility/Console.h>


namespace artekmed {

    namespace compute {

        void ProgramWrapper::PrintProgramWarning(const std::string &message) const
        {
            PrintWarning("[%s] %s\n", GetProgramName().c_str(), message.c_str());
        }

        bool ProgramWrapper::CompileProgram(const char *const kernel_code)
        {
            if (compiled_) {
                return true;
            }

            if (!cv::ocl::haveOpenCL()) {
                PrintProgramWarning("OpenCL is not available.");
                return false;
            }

            auto context = &cv::ocl::Context::getDefault();

            if (kernel_code != NULL) {
                cv::ocl::ProgramSource programSource(kernel_code);
                // Compile the kernel code
                cv::String errmsg;
                cv::String buildopt = GenerateBuildOptions();
                m_program = context->getProg(programSource, buildopt, errmsg);

                // validate kernel ??
            }


            compiled_ = true;
            return true;
        }

        void ProgramWrapper::ReleaseProgram()
        {
            if (compiled_) {
                compiled_ = false;
            }
        }


    }    // namespace artekmed::compute

}    // namespace artekmed
