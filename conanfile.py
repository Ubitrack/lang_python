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
    options = { "python": "ANY",
                "workspaceBuild" : [True, False],}
    default_options = { "python":"python", 
                        "workspaceBuild" : False,}

    short_paths = True
    settings = "os", "compiler", "build_type", "arch"
    generators = "cmake"

    # all sources are deployed with the package
    exports_sources = "cmake/*", "include/*", "doc/*", "lib/*", "src/*", "tests/*", "CMakeLists.txt", "setup.py.in"

    def requirements(self):
        userChannel = "ubitrack/stable"
        if self.options.workspaceBuild:
            userChannel = "local/dev"

        self.requires("ubitrack_core/%s@%s" % (self.version, userChannel))
        self.requires("ubitrack_dataflow/%s@%s" % (self.version, userChannel))
        self.requires("ubitrack_vision/%s@%s" % (self.version, userChannel))

        self.requires("ubitrack_hapticcalibration/%s@%s" % (self.version, userChannel))
        self.requires("ubitrack_facade/%s@%s" % (self.version, userChannel))
        self.requires("ubitrack_component_core/%s@%s" % (self.version, userChannel))
        self.requires("pybind11/[>=2.2.1]@camposs/stable")

    def configure(self):
        self.options['Boost'].without_python = True
        self.options['python_dev_config'].python = self.options.python

    # def imports(self):
    #     self.copy(pattern="*.dll", dst="bin", src="bin") # From bin to bin
    #     self.copy(pattern="*.dylib*", dst="lib", src="lib") 
    #     self.copy(pattern="*.so*", dst="lib", src="lib") 
       
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