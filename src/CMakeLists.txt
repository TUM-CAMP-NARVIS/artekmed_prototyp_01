set(UBITRACK_APPLICATION_NAME artekmed_p1)



# some black magic to automatically create a header file from shader files
find_program(SHADERENCODER EncodeShader PATHS ${CONAN_OPEN3D_ROOT}/bin)
message(STATUS "Open3D Shader Encoder: ${SHADERENCODER}")

file(GLOB SHADER_FILES "GLSL/*.glsl")
add_custom_command(
        OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/UbitrackShader.h
        COMMAND ${SHADERENCODER} UbitrackShader.h
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
        DEPENDS ${SHADER_FILES} ${SHADERENCODER}
)
foreach(file ${SHADER_FILES})
  add_custom_command(
          OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/UbitrackShader.h
          COMMAND ${SHADERENCODER} UbitrackShader.h ${file}
          WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
          DEPENDS ${SHADER_FILES} ${SHADERENCODER}
          APPEND
  )
endforeach(file)
add_custom_target(ShaderFileTarget ALL
        DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/UbitrackShader.h)
set_target_properties(ShaderFileTarget PROPERTIES FOLDER "Custom")

source_group("Source Files\\GLSL" FILES ${SHADER_FILES})



# All cpp files
set(UBITRACK_SRCS
  "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/PointCloudProcessing.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/UbitrackImage.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/UbitrackImageShader.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/UbitrackImageRenderer.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/UbitrackBaseConnector.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/UbitrackPointCloudConnector.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/UbitrackVisualizer.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/UbitrackPointCloudVisualizer.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/UbitrackViewControl.cpp"
)

# All header files
set(UBITRACK_HEADERS
  "${CMAKE_SOURCE_DIR}/include/artekmed/PointCloudProcessing.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/UbitrackHelpers.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/UbitrackImage.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/UbitrackImageShader.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/UbitrackImageRenderer.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/UbitrackBaseConnector.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/UbitrackPointCloudConnector.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/UbitrackVisualizer.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/UbitrackPointCloudVisualizer.h"
  "${CMAKE_SOURCE_DIR}/include/artekmed/UbitrackViewControl.h"
        ../include/artekmed/UbitrackHelpers.h)

if(MSVC)
  set(UBITRACK_COMPILE_DEFINITIONS ${UBITRACK_COMPILE_DEFINITIONS} "BOOST_ALL_DYN_LINK")
endif(MSVC)

# UTCORE library definition
add_executable(${UBITRACK_APPLICATION_NAME} ${UBITRACK_SRCS} ${UBITRACK_HEADERS})
add_dependencies(${UBITRACK_APPLICATION_NAME} ShaderFileTarget)

target_link_libraries(${UBITRACK_APPLICATION_NAME}
        CONAN_PKG::ubitrack_core
        CONAN_PKG::ubitrack_vision
        CONAN_PKG::ubitrack_facade
        CONAN_PKG::open3d
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
