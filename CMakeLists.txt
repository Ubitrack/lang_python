SET(PROJECT_NAME "ubitrack_python")
PROJECT(${PROJECT_NAME})

cmake_minimum_required(VERSION 3.4.0)


if(EXISTS ${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
  include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
  conan_basic_setup(KEEP_RPATHS)
else()
  message(WARNING "The file conanbuildinfo.cmake doesn't exist, you have to run conan install first")
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

# Add a CMake parameter for choosing a desired Python version
set(UBITRACK_PYTHON_VERSION "" CACHE STRING "Python version to use for compiling the ubitrack python library")

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

include(GNUInstallDirs)
include(CheckCXXCompilerFlag)

# Set a default build configuration if none is specified. 'MinSizeRel' produces the smallest binaries
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'MinSizeRel' as none was specified.")
  set(CMAKE_BUILD_TYPE MinSizeRel CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()
string(TOUPPER "${CMAKE_BUILD_TYPE}" U_CMAKE_BUILD_TYPE)


include(cmake/UbitrackConfig)
include(cmake/PythonModuleBuilder.cmake)

SET( UBITRACK_COMPILE_FLAGS ${UBITRACK_COMPILE_FLAGS} "-D${PROJECT_NAME}_EXPORTS" )
SET( ${PROJECT_NAME}_SOURCE_DIR ${${PROJECT_NAME}_SOURCE_DIR} )

set(UBITRACK_EXTRA_LIBS)

# Version of ${PROJECT_NAME}
SET( ${PROJECT_NAME}_MAJOR_VERSION 1 )
SET( ${PROJECT_NAME}_MINOR_VERSION 3 )
SET( ${PROJECT_NAME}_BUILD_VERSION 0 )

# OpenGL
SET(HAVE_OPENGL 0)
find_package(OpenGL)
IF(OPENGL_FOUND)
  add_definitions(-DHAVE_OPENGL)
  SET(HAVE_OPENGL 1)
  MESSAGE(STATUS "Found OpenGL: ${OPENGL_INCLUDE_DIR} - ${OPENGL_LIBRARIES}")
  INCLUDE_DIRECTORIES(${OPENGL_INCLUDE_DIR})
  set(UBITRACK_EXTRA_LIBS ${UBITRACK_EXTRA_LIBS} ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY})
ENDIF(OPENGL_FOUND)

set(Python_ADDITIONAL_VERSIONS 3.4 3.5 3.6 3.7)
include(cmake/UbiTrackDetectPython.cmake)

# The above sometimes returns version numbers like "3.4.3+"; the "+" must be removed for the next lines to work
string(REPLACE "+" "" PYTHONLIBS_VERSION_STRING "+${PYTHONLIBS_VERSION_STRING}")

if (CMAKE_CXX_COMPILER_ID MATCHES "Clang" OR CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  CHECK_CXX_COMPILER_FLAG("-std=c++14" HAS_CPP14_FLAG)
  CHECK_CXX_COMPILER_FLAG("-std=c++11" HAS_CPP11_FLAG)

  if (HAS_CPP14_FLAG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
  elseif (HAS_CPP11_FLAG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
  else()
    message(FATAL_ERROR "Unsupported compiler -- at least C++11 support is needed!")
  endif()

  # Enable link time optimization and set the default symbol
  # visibility to hidden (very important to obtain small binaries)
  if (NOT ${U_CMAKE_BUILD_TYPE} MATCHES DEBUG)
    # Default symbol visibility
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")

    # Check for Link Time Optimization support
    CHECK_CXX_COMPILER_FLAG("-flto" HAS_LTO_FLAG)
    if (HAS_LTO_FLAG)
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -flto")
    endif()
  endif()
endif()

# Include path for Python header files
include_directories(${PYTHON_INCLUDE_DIR})

# Include path for pybind11 header files -- this may need to be changed depending on your setup
# include_directories(${PROJECT_SOURCE_DIR}/pybind11/include)


set(UBITRACK_EXTRA_LIBS ${UBITRACK_EXTRA_LIBS} ${PYTHON_LIBRARIES})

SET(MODULE_INSTALL_PATH ${PYTHON_PACKAGE_DIR})
SET(MODULE_LIBS ${CONAN_LIBS} ${UBITRACK_EXTRA_LIBS})


MESSAGE(STATUS "Python ${PYTHON_VERSION_FULL} Executable: ${PYTHON_EXECUTABLE}")   
MESSAGE(STATUS "Python Include: ${PYTHON_INCLUDE_DIR} Numpy: ${PYTHON_NUMPY_INCLUDE_DIR}")   
MESSAGE(STATUS "Python Lib: ${PYTHON_LIBRARIES}")   
MESSAGE(STATUS "Install Directory: ${CMAKE_INSTALL_PREFIX}")   

# if (PYTHON_EXECUTABLE)
#   set(SETUP_PY_IN "${CMAKE_CURRENT_SOURCE_DIR}/setup.py.in")
#   set(SETUP_PY    "${CMAKE_CURRENT_BINARY_DIR}/setup.py")
#   set(DEPS        "${CMAKE_CURRENT_SOURCE_DIR}/lib/ubitrack/__init__.py")
#   set(OUTPUT      "${CMAKE_CURRENT_BINARY_DIR}/build")

#   configure_file(${SETUP_PY_IN} ${SETUP_PY})

# 	add_custom_command(
# 	  OUTPUT ${OUTPUT}/timestamp
# 	  COMMAND ${PYTHON_EXECUTABLE} setup.py build
# 	  COMMAND ${CMAKE_COMMAND} -E touch ${OUTPUT}/timestamp
# 	  DEPENDS ${DEPS}
# 	)
# 	add_custom_target(pybuild ALL DEPENDS ${OUTPUT}/timestamp )

#   install(CODE "execute_process(COMMAND ${PYTHON_EXECUTABLE} ${SETUP_PY} install --prefix=${CMAKE_INSTALL_PREFIX})")
# endif(PYTHON_EXECUTABLE)

INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include")

add_subdirectory(src)

