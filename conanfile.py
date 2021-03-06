from conans import ConanFile, CMake
from conans import tools
from conans.tools import os_info, SystemPackageTool
import os, sys
import sysconfig
from io import StringIO


class UbitrackCoreConan(ConanFile):
    name = "ubitrack_lang_python"
    version = "1.3.0"

    description = "Ubitrack Facade Modern Python Bindings"
    url = "https://github.com/Ubitrack/lang_python.git"
    license = "GPL"
    options = {"python": "ANY",}
    default_options = ("python=python",)

    short_paths = True
    settings = "os", "compiler", "build_type", "arch"
    generators = "cmake"
    requires = (
        "pybind11/[>=2.2.1]@camposs/stable", # only until we manage to upgrade to boost versions >= 1.63.0
        "ubitrack_core/%s@ubitrack/stable" % version,
        "ubitrack_vision/%s@ubitrack/stable" % version,
        "ubitrack_dataflow/%s@ubitrack/stable" % version,
        "ubitrack_hapticcalibration/%s@ubitrack/stable" % version,
        "ubitrack_facade/%s@ubitrack/stable" % version,
        "ubitrack_component_core/%s@ubitrack/stable" % version, # for testing
       )

    # all sources are deployed with the package
    exports_sources = "cmake/*", "include/*", "doc/*", "lib/*", "src/*", "tests/*", "CMakeLists.txt", "setup.py.in"

    def configure(self):
        self.options['Boost'].without_python = True

    def imports(self):
        self.copy(pattern="*.dll", dst="bin", src="bin") # From bin to bin
        self.copy(pattern="*.dylib*", dst="lib", src="lib") 
        self.copy(pattern="*.so*", dst="lib", src="lib") 
       
    def build(self):
        cmake = CMake(self)
        cmake.verbose = True
        self.output.info("python executable: %s (%s)" % (self.deps_user_info["python_dev_config"].python_exec,
                                                         self.deps_user_info["python_dev_config"].python_version))
        cmake.definitions['PYTHON_EXECUTABLE'] = self.deps_user_info["python_dev_config"].python_exec
        cmake.definitions['PYTHON_VERSION_STRING'] = self.deps_user_info["python_dev_config"].python_version
        if self.settings.os == "Macos":
            cmake.definitions['CMAKE_FIND_FRAMEWORK'] = "LAST"
        cmake.configure()
        cmake.build()
        cmake.install()

    def package_info(self):
        self.env_info.PYTHONPATH.append(os.path.join(self.package_folder, 'lib', 'python'))