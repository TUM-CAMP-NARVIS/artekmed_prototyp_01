from conans import ConanFile, CMake
from conans import tools
from conans.tools import os_info, SystemPackageTool
import os, sys
import sysconfig
from io import StringIO

class UbitrackCoreConan(ConanFile):
    name = "ubitrack_lang_python"
    version = "1.3.0"

    description = "Example for using Ubitrack Simple Facade"
    url = "https://github.com/Ubitrack/example_basic_facade.git"
    license = "GPL"

    short_paths = True
    settings = "os", "compiler", "build_type", "arch"
    generators = "cmake", "ubitrack_virtualenv_generator"

    requires = (
        "ubitrack/%s@ubitrack/stable" % version,
        "ubitrack_component_vision_aruco/%s@ubitrack/stable" % version,
        "ubitrack_device_camera_zed/%s@ubitrack/stable" % version,
        "ubitrack_device_camera_realsense/%s@ubitrack/stable" % version,
        "ubitrack_device_comm_videostream/%s@ubitrack/stable" % version,
        "ubitrack_tools_trackman/1.0@ubitrack/stable",
        "eigen/[>=3.3.4]@camposs/stable",
        "open3d/0.5.0@camposs/stable",
        "imgui/1.66@camposs/stable",
        "jsoncpp/[>=1.7.1]@camposs/stable",
        # "folly/2018.11.12.00@camposs/stable",
       )

    # all sources are deployed with the package
    exports_sources = "cmake/*", "include/*", "config/*", "src/*", "CMakeLists.txt"

    def configure(self):
        self.options['ubitrack_vision'].opengl_extension_wrapper = "glad"
        self.options['ubitrack'].with_default_camera=True
        self.options['ubitrack'].with_camera_zed=True # this might require to set sdk-root option !!
        self.options['ubitrack'].with_camera_realsense=True

        if self.settings.os == "Linux":
            self.options["opencv"].with_gtk = True
            self.options['ubitrack_core'].enable_tracing = True

        # self.options['ubitrack'].with_device_videostream=True
        # self.options['ubitrack'].with_haptic_calibration=True
        # if self.settings.os == "Windows":
        #     self.options['ubitrack'].with_camera_flycapture=True


    def imports(self):
        self.copy(src="bin", pattern="*.dll", dst="./bin") # Copies all dll files from packages bin folder to my "bin" folder
        self.copy(src="lib", pattern="*.dll", dst="./bin") # Copies all dll files from packages bin folder to my "bin" folder
        self.copy(src="lib", pattern="*.dylib*", dst="./lib") # Copies all dylib files from packages lib folder to my "lib" folder
        self.copy(src="lib", pattern="*.so*", dst="./lib") # Copies all so files from packages lib folder to my "lib" folder
        self.copy(src="bin", pattern="ut*", dst="./bin") # Copies all applications
        self.copy(src="bin", pattern="log4cpp.conf", dst="./") # copy a logging config template
        self.copy(src="share/Ubitrack", pattern="*.*", dst="./share/Ubitrack") # copy all shared ubitrack files 
       
    def build(self):
        cmake = CMake(self)
        cmake.verbose = True
        cmake.configure()
        cmake.build()
        cmake.install()
