
include_directories( "${CMAKE_SOURCE_DIR}/include" )

# some black magic to automatically create a header file from shader files
find_program(SHADERENCODER EncodeShader)
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




# All cpp files for DEMO
SET(DEMO_SRCS
        "${CMAKE_CURRENT_SOURCE_DIR}/main.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/UbitrackImage.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/UbitrackImageShader.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/UbitrackImageRenderer.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/UbitrackBaseConnector.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/UbitrackSingleCameraConnector.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/UbitrackVisualizer.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/UbitrackSingleCameraVisualizer.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/UbitrackViewControl.cpp"
)


# All header files for DEMO
SET(DEMO_HEADERS
        "${CMAKE_SOURCE_DIR}/include/basic_facade_demo/UbitrackImage.h"
        "${CMAKE_SOURCE_DIR}/include/basic_facade_demo/UbitrackImageShader.h"
        "${CMAKE_SOURCE_DIR}/include/basic_facade_demo/UbitrackImageRenderer.h"
        "${CMAKE_SOURCE_DIR}/include/basic_facade_demo/UbitrackBaseConnector.h"
        "${CMAKE_SOURCE_DIR}/include/basic_facade_demo/UbitrackSingleCameraConnector.h"
        "${CMAKE_SOURCE_DIR}/include/basic_facade_demo/UbitrackVisualizer.h"
        "${CMAKE_SOURCE_DIR}/include/basic_facade_demo/UbitrackSingleCameraVisualizer.h"
        "${CMAKE_SOURCE_DIR}/include/basic_facade_demo/UbitrackViewControl.h"
)

ADD_EXECUTABLE(${PROJECT_NAME} ${DEMO_SRCS} ${DEMO_HEADERS})
add_dependencies(${PROJECT_NAME} ShaderFileTarget)

TARGET_LINK_LIBRARIES( ${PROJECT_NAME} ${CONAN_LIBS} ${UBITRACK_EXTRA_LIBS} )
# Apply Target Properties
if(MSVC)
  if(CMAKE_CROSSCOMPILING)
    set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:secchk")
  endif()
  set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:libc /DEBUG")
endif()

foreach(_flag ${UBITRACK_COMPILE_FLAGS})
  set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "${_flag}")
endforeach()
foreach(_flag ${UBITRACK_LINK_FLAGS})
  set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS "${_flag}")
endforeach()
foreach(_flag ${UBITRACK_LINK_FLAGS_DEBUG})
  set_target_properties(${PROJECT_NAME} PROPERTIES LINK_FLAGS_DEBUG "${_flag}")
endforeach()
foreach(_symb ${UBITRACK_DEFINES})
  set_target_properties(${PROJECT_NAME} PROPERTIES DEFINE_SYMBOL ${_symb})
endforeach()

# set compiler Definitions
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_DEFINITIONS "${UBITRACK_COMPILE_DEFINITIONS}")


set_target_properties(${PROJECT_NAME} PROPERTIES
  OUTPUT_NAME "${PROJECT_NAME}${UBITRACK_DLLVERSION}"
  DEBUG_POSTFIX "${UBITRACK_DEBUG_POSTFIX}"
  ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_INSTALL_LIBDIR}
  )

if(NOT ANDROID)
  set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${UBITRACK_LIBVERSION}
    SOVERSION ${UBITRACK_SOVERSION}
    )
endif(NOT ANDROID)

# Installation of libraries, headers and documents
install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  )