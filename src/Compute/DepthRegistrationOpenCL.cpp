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

#include <fstream>

#define CL_USE_DEPRECATED_OPENCL_1_2_APIS
#define CL_USE_DEPRECATED_OPENCL_2_0_APIS

#ifdef OPENCL_ICD_LOADER_IS_OLD
#define CL_USE_DEPRECATED_OPENCL_1_1_APIS
#include <CL/cl.h>
#ifdef CL_VERSION_1_2
#undef CL_VERSION_1_2
#endif //CL_VERSION_1_2
#endif //OPENCL_ICD_LOADER_IS_OLD

#include <CL/cl.hpp>

#include "artekmed/Compute/OCLKernels.h"
#include "artekmed/Compute/DepthRegistrationOpenCL.h"


#include <log4cpp/Category.hh>
static log4cpp::Category& logger(log4cpp::Category::getInstance("ArtekmedP1.Compute.DepthRegistrationOCL"));

#define ENABLE_PROFILING_CL


const char *getErrorString(cl_int error)
{
  switch(error){
    // run-time and JIT compiler errors
    case 0: return "CL_SUCCESS";
    case -1: return "CL_DEVICE_NOT_FOUND";
    case -2: return "CL_DEVICE_NOT_AVAILABLE";
    case -3: return "CL_COMPILER_NOT_AVAILABLE";
    case -4: return "CL_MEM_OBJECT_ALLOCATION_FAILURE";
    case -5: return "CL_OUT_OF_RESOURCES";
    case -6: return "CL_OUT_OF_HOST_MEMORY";
    case -7: return "CL_PROFILING_INFO_NOT_AVAILABLE";
    case -8: return "CL_MEM_COPY_OVERLAP";
    case -9: return "CL_IMAGE_FORMAT_MISMATCH";
    case -10: return "CL_IMAGE_FORMAT_NOT_SUPPORTED";
    case -11: return "CL_BUILD_PROGRAM_FAILURE";
    case -12: return "CL_MAP_FAILURE";
    case -13: return "CL_MISALIGNED_SUB_BUFFER_OFFSET";
    case -14: return "CL_EXEC_STATUS_ERROR_FOR_EVENTS_IN_WAIT_LIST";
    case -15: return "CL_COMPILE_PROGRAM_FAILURE";
    case -16: return "CL_LINKER_NOT_AVAILABLE";
    case -17: return "CL_LINK_PROGRAM_FAILURE";
    case -18: return "CL_DEVICE_PARTITION_FAILED";
    case -19: return "CL_KERNEL_ARG_INFO_NOT_AVAILABLE";

          // compile-time errors
    case -30: return "CL_INVALID_VALUE";
    case -31: return "CL_INVALID_DEVICE_TYPE";
    case -32: return "CL_INVALID_PLATFORM";
    case -33: return "CL_INVALID_DEVICE";
    case -34: return "CL_INVALID_CONTEXT";
    case -35: return "CL_INVALID_QUEUE_PROPERTIES";
    case -36: return "CL_INVALID_COMMAND_QUEUE";
    case -37: return "CL_INVALID_HOST_PTR";
    case -38: return "CL_INVALID_MEM_OBJECT";
    case -39: return "CL_INVALID_IMAGE_FORMAT_DESCRIPTOR";
    case -40: return "CL_INVALID_IMAGE_SIZE";
    case -41: return "CL_INVALID_SAMPLER";
    case -42: return "CL_INVALID_BINARY";
    case -43: return "CL_INVALID_BUILD_OPTIONS";
    case -44: return "CL_INVALID_PROGRAM";
    case -45: return "CL_INVALID_PROGRAM_EXECUTABLE";
    case -46: return "CL_INVALID_KERNEL_NAME";
    case -47: return "CL_INVALID_KERNEL_DEFINITION";
    case -48: return "CL_INVALID_KERNEL";
    case -49: return "CL_INVALID_ARG_INDEX";
    case -50: return "CL_INVALID_ARG_VALUE";
    case -51: return "CL_INVALID_ARG_SIZE";
    case -52: return "CL_INVALID_KERNEL_ARGS";
    case -53: return "CL_INVALID_WORK_DIMENSION";
    case -54: return "CL_INVALID_WORK_GROUP_SIZE";
    case -55: return "CL_INVALID_WORK_ITEM_SIZE";
    case -56: return "CL_INVALID_GLOBAL_OFFSET";
    case -57: return "CL_INVALID_EVENT_WAIT_LIST";
    case -58: return "CL_INVALID_EVENT";
    case -59: return "CL_INVALID_OPERATION";
    case -60: return "CL_INVALID_GL_OBJECT";
    case -61: return "CL_INVALID_BUFFER_SIZE";
    case -62: return "CL_INVALID_MIP_LEVEL";
    case -63: return "CL_INVALID_GLOBAL_WORK_SIZE";
    case -64: return "CL_INVALID_PROPERTY";
    case -65: return "CL_INVALID_IMAGE_DESCRIPTOR";
    case -66: return "CL_INVALID_COMPILER_OPTIONS";
    case -67: return "CL_INVALID_LINKER_OPTIONS";
    case -68: return "CL_INVALID_DEVICE_PARTITION_COUNT";

          // extension errors
    case -1000: return "CL_INVALID_GL_SHAREGROUP_REFERENCE_KHR";
    case -1001: return "CL_PLATFORM_NOT_FOUND_KHR";
    case -1002: return "CL_INVALID_D3D10_DEVICE_KHR";
    case -1003: return "CL_INVALID_D3D10_RESOURCE_KHR";
    case -1004: return "CL_D3D10_RESOURCE_ALREADY_ACQUIRED_KHR";
    case -1005: return "CL_D3D10_RESOURCE_NOT_ACQUIRED_KHR";
    default: return "Unknown OpenCL error";
  }
}


#define CL_FILENAME (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define PRINT_CL_ERROR(expr, err) LOG4CPP_ERROR(logger,  "[" << CL_FILENAME << "](" << __LINE__ << ") " << expr << " failed: " << getErrorString(err));
#define CHECK_CL_PARAM(expr) do { cl_int err = CL_SUCCESS; (expr); if (err != CL_SUCCESS) { PRINT_CL_ERROR(#expr, err); return false; } } while(0)
#define CHECK_CL_RETURN(expr) do { cl_int err = (expr); if (err != CL_SUCCESS) { PRINT_CL_ERROR(#expr, err); return false; } } while(0)
#define CHECK_CL_ON_FAIL(expr, on_fail) do { cl_int err = (expr); if (err != CL_SUCCESS) { PRINT_CL_ERROR(#expr, err); on_fail; return false; } } while(0)


struct DepthRegistrationOpenCL::OCLData
{
  cl::Context context;
  cl::Device device;

  cl::Program program;
  cl::CommandQueue queue;

  cl::Kernel kernelGeneratePoints;
  cl::Kernel kernelSetZero;

  size_t sizeDepth;
  size_t sizePoints;

  cl::Buffer bufferDepth;
  cl::Buffer bufferPoints;

  cl::Buffer bufferOutput;
  unsigned char *dataOutput;

#ifdef ENABLE_PROFILING_CL
  std::vector<double> timings;
  int count;
#endif
};

DepthRegistrationOpenCL::DepthRegistrationOpenCL()
  : DepthRegistration()
{
  data = new OCLData;
}

DepthRegistrationOpenCL::~DepthRegistrationOpenCL()
{
  delete data;
}

void getDevices(const std::vector<cl::Platform> &platforms, std::vector<cl::Device> &devices)
{
  devices.clear();
  for(size_t i = 0; i < platforms.size(); ++i)
  {
    const cl::Platform &platform = platforms[i];

    std::vector<cl::Device> devs;
    if(platform.getDevices(CL_DEVICE_TYPE_ALL, &devs) != CL_SUCCESS)
    {
      continue;
    }

    devices.insert(devices.end(), devs.begin(), devs.end());
  }
}

std::string deviceString(cl::Device &dev)
{
  std::string devName, devVendor, devType;
  cl_device_type devTypeID;
  dev.getInfo(CL_DEVICE_NAME, &devName);
  dev.getInfo(CL_DEVICE_VENDOR, &devVendor);
  dev.getInfo(CL_DEVICE_TYPE, &devTypeID);

  switch(devTypeID)
  {
  case CL_DEVICE_TYPE_CPU:
    devType = "CPU";
    break;
  case CL_DEVICE_TYPE_GPU:
    devType = "GPU";
    break;
  case CL_DEVICE_TYPE_ACCELERATOR:
    devType = "ACCELERATOR";
    break;
  default:
    devType = "CUSTOM/UNKNOWN";
  }

  return devName + " (" + devType + ")[" + devVendor + ']';
}

bool selectDevice(std::vector<cl::Device> &devices, cl::Device &device, const int deviceId = -1)
{
  if(deviceId != -1 && devices.size() > (size_t)deviceId)
  {
    device = devices[deviceId];
    return true;
  }

  bool selected = false;
  cl_device_type selectedType = 0;

  for(size_t i = 0; i < devices.size(); ++i)
  {
    cl::Device &dev = devices[i];
    cl_device_type devTypeID;
    dev.getInfo(CL_DEVICE_TYPE, &devTypeID);

    if(!selected || (selectedType != CL_DEVICE_TYPE_GPU && devTypeID == CL_DEVICE_TYPE_GPU))
    {
      selectedType = devTypeID;
      selected = true;
      device = dev;
    }
  }
  return selected;
}

bool DepthRegistrationOpenCL::init(const int deviceId)
{
  std::string sourceCode(open3d::glsl::DepthRegistration);

  std::vector<cl::Platform> platforms;
  CHECK_CL_RETURN(cl::Platform::get(&platforms));

  if(platforms.empty())
  {
    LOG4CPP_ERROR(logger, "no opencl platforms found.");
    return false;
  }

  std::vector<cl::Device> devices;
  getDevices(platforms, devices);

  LOG4CPP_INFO(logger, "devices:");
  for(size_t i = 0; i < devices.size(); ++i)
  {
    LOG4CPP_INFO(logger, "  " << i << ": " << deviceString(devices[i]));
  }

  if(!selectDevice(devices, data->device, deviceId))
  {
    LOG4CPP_ERROR(logger, "could not find any suitable device");
    return false;
  }
  LOG4CPP_INFO(logger, "selected device: " << deviceString(data->device));

  CHECK_CL_PARAM(data->context = cl::Context(data->device, NULL, NULL, NULL, &err));

  std::string options;
  generateOptions(options);

  cl::Program::Sources source(1, std::make_pair(sourceCode.c_str(), sourceCode.length()));
  CHECK_CL_PARAM(data->program = cl::Program(data->context, source, &err));

  CHECK_CL_ON_FAIL(data->program.build(options.c_str()),
                   LOG4CPP_ERROR(logger, "failed to build program: " << err);
                   LOG4CPP_ERROR(logger, "Build Options:\t" << data->program.getBuildInfo<CL_PROGRAM_BUILD_OPTIONS>(data->device));
                   LOG4CPP_ERROR(logger, "Build Log:\t" << data->program.getBuildInfo<CL_PROGRAM_BUILD_LOG>(data->device)));

#ifdef ENABLE_PROFILING_CL
  data->count = 0;
  CHECK_CL_PARAM(data->queue = cl::CommandQueue(data->context, data->device, CL_QUEUE_PROFILING_ENABLE, &err));
#else
  CHECK_CL_PARAM(data->queue = cl::CommandQueue(data->context, data->device, 0, &err));
#endif

  // for now size points == size depth.
  sizePoints = sizeDepth;

  data->sizeDepth = sizeDepth.height * sizeDepth.width * sizeof(float);
  data->sizePoints = sizePoints.height * sizePoints.width * sizeof(cl_float3);

  CHECK_CL_PARAM(data->bufferDepth = cl::Buffer(data->context, CL_MEM_READ_ONLY, data->sizeDepth, NULL, &err));
  CHECK_CL_PARAM(data->bufferPoints = cl::Buffer(data->context, CL_MEM_READ_WRITE, data->sizePoints, NULL, &err));
  CHECK_CL_PARAM(data->bufferOutput = cl::Buffer(data->context, CL_MEM_WRITE_ONLY | CL_MEM_ALLOC_HOST_PTR, data->sizePoints, NULL, &err));

  CHECK_CL_PARAM(data->kernelSetZero = cl::Kernel(data->program, "setZero", &err));
  CHECK_CL_RETURN(data->kernelSetZero.setArg(0, data->bufferPoints));

  CHECK_CL_PARAM(data->kernelGeneratePoints = cl::Kernel(data->program, "depthToPoints", &err));
  CHECK_CL_RETURN(data->kernelGeneratePoints.setArg(0, data->bufferDepth));
  CHECK_CL_RETURN(data->kernelGeneratePoints.setArg(1, data->bufferPoints));

  CHECK_CL_PARAM(data->dataOutput = (unsigned char *)data->queue.enqueueMapBuffer(data->bufferOutput, CL_TRUE, CL_MAP_READ, 0, data->sizePoints, NULL, NULL, &err));
  return true;
}


bool DepthRegistrationOpenCL::depthToPoints(const cv::Mat &depth, cv::Mat &points)
{
  cl::Event eventRead;
  std::vector<cl::Event> eventZero(2), eventGeneratePoints(1);
  cl::NDRange range(sizePoints.height * sizePoints.width);

  CHECK_CL_RETURN(data->queue.enqueueWriteBuffer(data->bufferDepth, CL_FALSE, 0, data->sizeDepth, depth.data, NULL, &eventZero[0]));
  CHECK_CL_RETURN(data->queue.enqueueNDRangeKernel(data->kernelSetZero, cl::NullRange, range, cl::NullRange, NULL, &eventZero[1]));

  CHECK_CL_RETURN(data->queue.enqueueNDRangeKernel(data->kernelGeneratePoints, cl::NullRange, range, cl::NullRange, &eventZero, &eventGeneratePoints[0]));

  CHECK_CL_RETURN(data->queue.enqueueReadBuffer(data->bufferPoints, CL_FALSE, 0, data->sizePoints, data->dataOutput, &eventGeneratePoints, &eventRead));

  CHECK_CL_RETURN(eventRead.wait());

  points = cv::Mat(sizePoints, CV_32FC3, data->dataOutput);

#ifdef ENABLE_PROFILING_CL
  if(data->count == 0)
  {
    data->timings.clear();
    data->timings.resize(7, 0.0);
  }

  data->timings[0] += eventZero[0].getProfilingInfo<CL_PROFILING_COMMAND_END>() - eventZero[0].getProfilingInfo<CL_PROFILING_COMMAND_START>();
  data->timings[1] += eventZero[1].getProfilingInfo<CL_PROFILING_COMMAND_END>() - eventZero[1].getProfilingInfo<CL_PROFILING_COMMAND_START>();
  data->timings[2] += eventGeneratePoints[0].getProfilingInfo<CL_PROFILING_COMMAND_END>() - eventGeneratePoints[0].getProfilingInfo<CL_PROFILING_COMMAND_START>();
  data->timings[3] += eventRead.getProfilingInfo<CL_PROFILING_COMMAND_END>() - eventRead.getProfilingInfo<CL_PROFILING_COMMAND_START>();

  if(++data->count == 30)
  {
    double sum = data->timings[0] + data->timings[1] + data->timings[2] + data->timings[3];
    LOG4CPP_INFO(logger, "writing depth:" << data->timings[0] / 100000000.0 << " ms.");
    LOG4CPP_INFO(logger, "setting zero: " << data->timings[1] / 100000000.0 << " ms.");
    LOG4CPP_INFO(logger, "generate points: " << data->timings[2] / 100000000.0 << " ms.");
    LOG4CPP_INFO(logger, "read points: " << data->timings[3] / 100000000.0 << " ms.");
    LOG4CPP_INFO(logger, "overall: " << sum / 100000000.0 << " ms.");
    data->count = 0;
  }
#endif
  return true;
}



void DepthRegistrationOpenCL::generateOptions(std::string &options) const
{
  std::ostringstream oss;
  oss.precision(16);
  oss << std::scientific;

  // Camera parameter upscaled depth
  oss << " -D fxD=" << cameraMatrixDepth.at<double>(0, 0) << "f";
  oss << " -D fyD=" << cameraMatrixDepth.at<double>(1, 1) << "f";
  oss << " -D cxD=" << cameraMatrixDepth.at<double>(0, 2) << "f";
  oss << " -D cyD=" << cameraMatrixDepth.at<double>(1, 2) << "f";
  oss << " -D fxDInv=" << (1.0 / cameraMatrixDepth.at<double>(0, 0)) << "f";
  oss << " -D fyDInv=" << (1.0 / cameraMatrixDepth.at<double>(1, 1)) << "f";

  // Size registered image
  oss << " -D heightP=" << sizePoints.height;
  oss << " -D widthP=" << sizePoints.width;

  // Size depth image
  oss << " -D heightD=" << sizeDepth.height;
  oss << " -D widthD=" << sizeDepth.width;

  oss << " -D scaleFactor=" << depthScale;
  oss << " -D DTYPE=float";

  options = oss.str();
}
