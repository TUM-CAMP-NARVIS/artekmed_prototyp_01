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
        "ubitrack_tools_trackman/1.0@ubitrack/stable",
       )

    # all sources are deployed with the package
    exports_sources = "cmake/*", "include/*", "config/*", "src/*", "CMakeLists.txt"

    def configure(self):
        self.options['ubitrack']:with_default_camera=True
        self.options['ubitrack']:with_haptic_calibration=True
        if self.sett

    def imports(self):
        self.copy(src="bin",pattern="*.dll", dst="./bin") # Copies all dll files from packages bin folder to my "bin" folder
        self.copy(src="lib",pattern="*.dll", dst="./bin") # Copies all dll files from packages bin folder to my "bin" folder
        self.copy(src="lib",pattern="*.dylib*", dst="./lib") # Copies all dylib files from packages lib folder to my "lib" folder
        self.copy(src="lib",pattern="*.so*", dst="./lib") # Copies all so files from packages lib folder to my "lib" folder
        self.copy(src="bin",pattern="ut*", dst="./bin") # Copies all applications
        self.copy(src="bin",pattern="log4cpp.conf", dst="./") # copy a logging config template
        self.copy(src="share/Ubitrack", ,pattern="*.*", dst="./share/Ubitrack") # copy all shared ubitrack files 
       
    def build(self):
        cmake = CMake(self)
        cmake.verbose = True
        cmake.configure()
        cmake.build()
        cmake.install()