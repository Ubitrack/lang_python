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

set(PYBIND11_PYTHON_VERSION ${UBITRACK_PYTHON_VERSION})
find_package(pybind11 REQUIRED)

MESSAGE(STATUS "PYTHON_PREFIX              ${PYTHON_PREFIX}")
MESSAGE(STATUS "PYTHON_LIBRARIES           ${PYTHON_LIBRARIES}")
MESSAGE(STATUS "PYTHON_INCLUDE_DIRS        ${PYTHON_INCLUDE_DIRS}")
MESSAGE(STATUS "PYTHON_MODULE_EXTENSION    ${PYTHON_MODULE_EXTENSION}")
MESSAGE(STATUS "PYTHON_MODULE_PREFIX       ${PYTHON_MODULE_PREFIX}")
MESSAGE(STATUS "PYTHON_SITE_PACKAGES       ${PYTHON_SITE_PACKAGES}")
MESSAGE(STATUS "PYTHON_IS_DEBUG            ${PYTHON_IS_DEBUG}")

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

# OpenCL
SET(HAVE_OPENCL 0)
find_package(OpenCL)
IF(OpenCL_FOUND)
  include_directories(${OpenCL_INCLUDE_DIR})
  SET(HAVE_OPENCL 1)
  MESSAGE(STATUS "Found OpenCL: ${OpenCL_INCLUDE_DIR} - ${OpenCL_LIBRARY}")
  set(UBITRACK_EXTRA_LIBS ${UBITRACK_EXTRA_LIBS} ${OpenCL_LIBRARY})
ENDIF(OpenCL_FOUND)

set(UBITRACK_EXTRA_LIBS ${UBITRACK_EXTRA_LIBS} ${PYTHON_LIBRARIES})
SET(MODULE_LIBS ${CONAN_LIBS} ${UBITRACK_EXTRA_LIBS})

include_directories("${CMAKE_SOURCE_DIR}/include")
add_subdirectory(src)

