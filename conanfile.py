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
       
    def build(self):
        cmake = CMake(self)
        cmake.verbose = True
        self.output.info("python executable: %s (%s)" % (self.python_exec.replace("\\", "/"), self.python_version))
        self.output.info("Python include: %s - library: %s" % (self.python_include, self.python_lib))
        self.output.info("Numpy include: %s" % self.numpy_include)
        cmake.definitions['PYTHON_EXECUTABLE'] = self.python_exec.replace("\\", "/")
        cmake.definitions['PYTHON_VERSION_STRING'] = self.python_version
        cmake.definitions['PYTHON_INCLUDE_DIR'] = self.python_include
        cmake.definitions['PYTHON_NUMPY_INCLUDE_DIR'] = self.numpy_include
        if self.settings.os == "Macos":
            cmake.definitions['CMAKE_FIND_FRAMEWORK'] = "LAST"
        cmake.configure()
        cmake.build()
        # cmake.install()

    def package(self):
        self.copy('ubitrack*.so', dst='lib/python', src='lib')
        self.copy('ubitrack*.pyd', dst='lib/python', src='lib')

    def package_info(self):
        self.env_info.PYTHONPATH.append(os.path.join(self.package_folder, 'lib', 'python'))

    @property
    def python_exec(self):
        try:
            pyexec = str(self.options.python)
            output = StringIO()
            self.run('{0} -c "import sys; print(sys.executable)"'.format(pyexec), output=output)
            return '"'+output.getvalue().strip().replace("\\","/")+'"'
        except:
            return ""
    
    _python_version = ""
    @property
    def python_version(self):
        cmd = "from sys import *; print('%d.%d' % (version_info[0],version_info[1]))"
        self._python_version = self._python_version or self.run_python_command(cmd)
        return self._python_version
      
    @property
    def python_include(self):
        pyinclude = self.get_python_path("include")
        if not os.path.exists(os.path.join(pyinclude, 'pyconfig.h')):
            return ""
        else:
            return pyinclude.replace('\\', '/')
    
    @property
    def python_lib(self):
        stdlib_dir = os.path.dirname(self.get_python_path("stdlib")).replace('\\', '/')
        return stdlib_dir
        
    @property
    def numpy_include(self):
        cmd = "import os; os.environ['DISTUTILS_USE_SDK']='1'; import numpy.distutils; print(numpy.distutils.misc_util.get_numpy_include_dirs()[0])" 
        return self.run_python_command(cmd)
                  
    def get_python_path(self, dir_name):
        cmd = "import sysconfig; print(sysconfig.get_path('{0}'))".format(dir_name)
        return self.run_python_command(cmd)   

    def run_python_command(self, cmd):
        pyexec = self.python_exec
        if pyexec:
            output = StringIO()
            self.run('{0} -c "{1}"'.format(pyexec, cmd), output=output)
            return output.getvalue().strip()
        else:
            return ""
