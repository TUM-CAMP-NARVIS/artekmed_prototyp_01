//
// Created by netlabs on 28.01.19.
//

#ifndef ARTEKMED_P1_OCLTESTPROCESSING_H
#define ARTEKMED_P1_OCLTESTPROCESSING_H

#include "artekmed/Compute/ProgramWrapper.h"

#include <Core/Geometry/PointCloud.h>


namespace artekmed {


    using namespace open3d;

    namespace compute {

        class OCLTestProcessor : public ProgramWrapper {

        public:
            ~OCLTestProcessor() override { Release(); }
        protected:
            bool Compile() final;
            void Release() final;

            std::string GenerateBuildOptions() override {
                return cv::format("-D dstT=float"); // fixed to float for now. - shoud be configurable"
            }

        public:
            OCLTestProcessor(const std::string& name)
                    : ProgramWrapper(name) {

            };

            void run_kernel();

        };

    }
}
#endif //ARTEKMED_P1_OCLPOINTCLOUDPROCESSING_H
