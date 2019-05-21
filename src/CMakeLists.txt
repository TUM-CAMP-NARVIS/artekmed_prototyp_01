set(UBITRACK_APPLICATION_NAME artekmed_p1)


# some black magic to automatically create a header file from shader files
find_program(SHADERENCODER EncodeShader PATHS ${CONAN_OPEN3D_ROOT}/bin)
message(STATUS "Open3D Shader Encoder: ${SHADERENCODER}")

file(GLOB SHADER_FILES "GLSL/*.glsl")
add_custom_command(
        OUTPUT ${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Shader/Shader.h
        COMMAND ${SHADERENCODER} Shader.h
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Shader/
        DEPENDS ${SHADER_FILES} ${SHADERENCODER}
)
foreach(file ${SHADER_FILES})
  add_custom_command(
          OUTPUT ${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Shader/Shader.h
          COMMAND ${SHADERENCODER} Shader.h ${file}
          WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Shader/
          DEPENDS ${SHADER_FILES} ${SHADERENCODER}
          APPEND
  )
endforeach(file)
add_custom_target(ShaderFileTarget ALL
        DEPENDS ${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Shader/Shader.h)
set_target_properties(ShaderFileTarget PROPERTIES FOLDER "Custom")

source_group("Source Files\\GLSL" FILES ${SHADER_FILES})


# OCL Sources inlined in headers
file(GLOB OCL_FILES "OCL/*.cl")
add_custom_command(
        OUTPUT ${CMAKE_SOURCE_DIR}/include/artekmed/Compute/OCLKernels.h
        COMMAND ${SHADERENCODER} OCLKernels.h
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/include/artekmed/Compute/
        DEPENDS ${OCL_FILES} ${SHADERENCODER}
)
foreach(file ${OCL_FILES})
  add_custom_command(
          OUTPUT ${CMAKE_SOURCE_DIR}/include/artekmed/Compute/OCLKernels.h
          COMMAND ${SHADERENCODER} OCLKernels.h ${file}
          WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/include/artekmed/Compute/
          DEPENDS ${OCL_FILES} ${SHADERENCODER}
          APPEND
  )
endforeach(file)
add_custom_target(OpenclFileTarget ALL
        DEPENDS ${CMAKE_SOURCE_DIR}/include/artekmed/Compute/OCLKernels.h)
set_target_properties(OpenclFileTarget PROPERTIES FOLDER "Custom")

source_group("Source Files\\OCL" FILES ${OCL_FILES})



# All cpp files
set(UBITRACK_SRCS
  "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/PointCloudProcessing.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/UbitrackPointCloudConnector.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/UbitrackPointCloudVisualizer.cpp"

  "${CMAKE_CURRENT_SOURCE_DIR}/Compute/DepthRegistration.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Compute/DepthRegistrationOpenCL.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Compute/ProgramWrapper.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Compute/OCLPointCloudProcessor.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Compute/OCLTestProcessor.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Ubitrack/CameraConnector.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Ubitrack/BaseConnector.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/UbitrackRenderManager.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Utility/UbitrackImage.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Shader/UbitrackImageShader.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Visualizer/UbitrackVisualizer.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Visualizer/UbitrackViewControl.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Shader/GeometryRenderer.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Shader/ImageMaskShader.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Shader/ImageShader.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Shader/NormalShader.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Shader/PhongShader.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Shader/PickingShader.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Shader/ShaderWrapper.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Shader/Simple2DShader.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Shader/SimpleBlackShader.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Shader/SimpleShader.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Utility/BoundingBox.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Utility/ColorMap.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Utility/DrawGeometry.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Utility/GLHelper.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Utility/PointCloudPicker.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Utility/SelectionPolygon.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Utility/SelectionPolygonVolume.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Visualizer/GuiController.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Visualizer/RenderOption.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Visualizer/RenderOptionWithEditing.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Visualizer/ViewControl.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Visualizer/ViewParameters.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Visualizer/VisualizerCallback.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Visualizer/VisualizerRender.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/Visualization/Visualizer/Visualizer.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/PointProcessing/RegionGrowing.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/PointProcessing/Common.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/PointProcessing/VoxelResampling.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/PointProcessing/Denoising.cpp"

        )

# All header files
set(UBITRACK_HEADERS
  "${CMAKE_SOURCE_DIR}/include/artekmed/EigenWrapper.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/PointCloudProcessing.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/UbitrackHelpers.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/UbitrackPointCloudConnector.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/UbitrackPointCloudVisualizer.h"

  "${CMAKE_SOURCE_DIR}/include/artekmed/Compute/DepthRegistration.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Compute/DepthRegistrationOpenCL.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Compute/ProgramWrapper.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Compute/cl.hpp"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Compute/OCLPointCloudProcessor.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Compute/OCLTestProcessor.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Ubitrack/CameraConnector.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Ubitrack/BaseConnector.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/UbitrackRenderManager.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Utility/UbitrackImage.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Shader/UbitrackImageShader.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Visualizer/UbitrackVisualizer.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Visualizer/UbitrackViewControl.h"


  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Visualization.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Shader/GeometryRenderer.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Shader/ImageMaskShader.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Shader/ImageShader.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Shader/NormalShader.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Shader/PhongShader.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Shader/PickingShader.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Shader/ShaderWrapper.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Shader/Simple2DShader.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Shader/SimpleBlackShader.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Shader/SimpleShader.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Utility/BoundingBox.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Utility/ColorMap.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Utility/DrawGeometry.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Utility/GLHelper.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Utility/PointCloudPicker.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Utility/SelectionPolygon.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Utility/SelectionPolygonVolume.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Visualizer/GuiController.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Visualizer/RenderOption.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Visualizer/RenderOptionWithEditing.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Visualizer/ViewControl.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Visualizer/ViewParameters.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Visualization/Visualizer/Visualizer.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/PointProcessing/RegionGrowing.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/PointProcessing/Common.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/PointProcessing/VoxelResampling.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/PointProcessing/Denoising.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/Utils/TemperatureColors.h"

        )

if(MSVC)
  set(UBITRACK_COMPILE_DEFINITIONS ${UBITRACK_COMPILE_DEFINITIONS} "BOOST_ALL_DYN_LINK")
endif(MSVC)

# UTCORE library definition
add_executable(${UBITRACK_APPLICATION_NAME} ${UBITRACK_SRCS} ${UBITRACK_HEADERS})
add_dependencies(${UBITRACK_APPLICATION_NAME} ShaderFileTarget)
add_dependencies(${UBITRACK_APPLICATION_NAME} OpenclFileTarget)

set_target_properties(${UBITRACK_APPLICATION_NAME} PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

target_link_libraries(${UBITRACK_APPLICATION_NAME}
        CONAN_PKG::ubitrack_core
        CONAN_PKG::ubitrack_vision
        CONAN_PKG::ubitrack_visualization
        CONAN_PKG::ubitrack_facade
        CONAN_PKG::open3d
        CONAN_PKG::jsoncpp
        CONAN_PKG::imgui
        OpenGL::GL
        OpenGL::GLU
        OpenCL::OpenCL
        )

target_include_directories(${UBITRACK_APPLICATION_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src>
        )

# Apply Target Properties
if(MSVC)
  if(CMAKE_CROSSCOMPILING)
    set_target_properties(${UBITRACK_APPLICATION_NAME} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:secchk")
  endif()
  set_target_properties(${UBITRACK_APPLICATION_NAME} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:libc /DEBUG")
endif()

ubitrack_set_target_properties(${UBITRACK_APPLICATION_NAME})
ubitrack_install_application(${UBITRACK_APPLICATION_NAME})
