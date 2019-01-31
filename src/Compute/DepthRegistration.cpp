/**
 * Copyright 2014 University of Bremen, Institute for Artificial Intelligence
 * Author: Thiemo Wiedemeyer <wiedemeyer@cs.uni-bremen.de>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <artekmed/Compute/DepthRegistration.h>

//#ifdef DEPTH_REG_CPU
//#include "depth_registration_cpu.h"
//#endif

//#ifdef DEPTH_REG_OPENCL
#include "artekmed/Compute/DepthRegistrationOpenCL.h"
//#endif

#include <log4cpp/Category.hh>
static log4cpp::Category& logger(log4cpp::Category::getInstance("ArtekmedP1.Compute.DepthRegistration"));

DepthRegistration::DepthRegistration()
{
}

DepthRegistration::~DepthRegistration()
{
}

bool DepthRegistration::init(const cv::Mat &cameraMatrixDepth, const cv::Size &sizeDepth, const float depthScale, const int deviceId)
{
    this->cameraMatrixDepth = cameraMatrixDepth;
    this->sizeDepth = sizeDepth;
    this->depthScale = depthScale;

    return init(deviceId);
}

DepthRegistration *DepthRegistration::New(Method method)
{
  if(method == DEFAULT)
  {
//#ifdef DEPTH_REG_OPENCL
    method = OPENCL;
//#elif defined DEPTH_REG_CPU
//    method = CPU;
//#endif
  }

  switch(method)
  {
  case DEFAULT:
    LOG4CPP_INFO(logger, "No default registration method available!");
    break;
  case CPU:
//#ifdef DEPTH_REG_CPU
//      LOG4CPP_INFO(logger, "Using CPU registration method!");
//    return new DepthRegistrationCPU();
//#else
//    LOG4CPP_INFO(logger, "CPU registration method not available!");
//    break;
//#endif
  case OPENCL:
//#ifdef DEPTH_REG_OPENCL
      LOG4CPP_INFO(logger, "Using OpenCL registration method!");
    return new DepthRegistrationOpenCL();
//#else
          LOG4CPP_INFO(logger, "OpenCL registration method not available!");
    break;
//#endif
  }
  return NULL;
}
