macro(add_python_module full_modname the_package MODULE_SRC_NAME)

MESSAGE(STATUS "python module ${full_modname} with sources: ${${MODULE_SRC_NAME}}")
ADD_LIBRARY(${full_modname} SHARED ${${MODULE_SRC_NAME}})

set_property(TARGET ${full_modname} PROPERTY PREFIX "")

if(WIN32)
    set_property(TARGET ${full_modname} PROPERTY SUFFIX ".pyd")
endif()

# make the name of debug libraries end in _d.
SET_TARGET_PROPERTIES( ${full_modname} PROPERTIES DEBUG_POSTFIX "_d" )

IF(UNIX) 
  SET_TARGET_PROPERTIES( ${full_modname} PROPERTIES OUTPUT_NAME ${full_modname} )
ENDIF(UNIX)

TARGET_LINK_LIBRARIES( ${full_modname} ${MODULE_LIBS})


if (WIN32)
  if (MSVC)
    # /bigobj is needed for bigger binding projects due to the limit to 64k
    # addressable sections. /MP enables multithreaded builds (relevant when
    # there are many files).
    set_target_properties(${full_modname} PROPERTIES COMPILE_FLAGS "/MP /bigobj ")

    if (NOT ${U_CMAKE_BUILD_TYPE} MATCHES DEBUG)
      # Enforce size-based optimization and link time code generation on MSVC
      # (~30% smaller binaries in experiments).
      set_target_properties(${full_modname} APPEND_STRING PROPERTY COMPILE_FLAGS "/Os /GL ")
      set_target_properties(${full_modname} APPEND_STRING PROPERTY LINK_FLAGS "/LTCG ")
    endif()
  endif()

  # .PYD file extension on Windows
  set_target_properties(${full_modname} PROPERTIES SUFFIX ".pyd")

  # Link against the Python shared library
  target_link_libraries(${full_modname} ${PYTHON_LIBRARY})
elseif (UNIX)
  # It's quite common to have multiple copies of the same Python version
  # installed on one's system. E.g.: one copy from the OS and another copy
  # that's statically linked into an application like Blender or Maya.
  # If we link our plugin library against the OS Python here and import it
  # into Blender or Maya later on, this will cause segfaults when multiple
  # conflicting Python instances are active at the same time (even when they
  # are of the same version).

  # Windows is not affected by this issue since it handles DLL imports
  # differently. The solution for Linux and Mac OS is simple: we just don't
  # link against the Python library. The resulting shared library will have
  # missing symbols, but that's perfectly fine -- they will be resolved at
  # import time.

  # .SO file extension on Linux/Mac OS
  set_target_properties(${full_modname} PROPERTIES SUFFIX ".so")

  # Strip unnecessary sections of the binary on Linux/Mac OS
  if(APPLE)
    set_target_properties(${full_modname} PROPERTIES MACOSX_RPATH ".")
    set_target_properties(${full_modname} PROPERTIES LINK_FLAGS "-undefined dynamic_lookup ")
    # if (NOT ${U_CMAKE_BUILD_TYPE} MATCHES DEBUG)
    #   add_custom_command(TARGET ${full_modname} POST_BUILD COMMAND strip -u -r ${PROJECT_BINARY_DIR}/${full_modname}.so)
    # endif()
  else()
    # if (NOT ${U_CMAKE_BUILD_TYPE} MATCHES DEBUG)
    #   add_custom_command(TARGET ${full_modname} POST_BUILD COMMAND strip ${PROJECT_BINARY_DIR}/${full_modname}.so)
    # endif()
  endif()
endif()

# Apply Target Properties
if(MSVC)
  if(CMAKE_CROSSCOMPILING)
    set_target_properties(${full_modname} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:secchk")
  endif()
  set_target_properties(${full_modname} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:libc /DEBUG")
endif()

foreach(_flag ${UBITRACK_COMPILE_FLAGS})
  set_target_properties(${full_modname} PROPERTIES COMPILE_FLAGS "${_flag}")
endforeach()
foreach(_flag ${UBITRACK_LINK_FLAGS})
  set_target_properties(${full_modname} PROPERTIES LINK_FLAGS "${_flag}")
endforeach()
foreach(_flag ${UBITRACK_LINK_FLAGS_DEBUG})
  set_target_properties(${full_modname} PROPERTIES LINK_FLAGS_DEBUG "${_flag}")
endforeach()
foreach(_symb ${UBITRACK_DEFINES})
  set_target_properties(${full_modname} PROPERTIES DEFINE_SYMBOL ${_symb})
endforeach()

# set compiler Definitions
set_target_properties(${full_modname} PROPERTIES COMPILE_DEFINITIONS "${UBITRACK_COMPILE_DEFINITIONS}")

# set fPIC
set_property(TARGET ${full_modname} PROPERTY POSITION_INDEPENDENT_CODE ON)

add_custom_command(
  OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/timestamp.${full_modname}
  COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${full_modname}> ${CMAKE_BINARY_DIR}/build/modules/${the_package}/$<TARGET_FILE_NAME:${full_modname}>
  COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/timestamp.${full_modname}
  DEPENDS ${DEPS}
  VERBATIM
)
add_custom_target(copy${full_modname} ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/timestamp.${full_modname} )
ADD_DEPENDENCIES(copy${full_modname} ${full_modname})
ADD_DEPENDENCIES(pybuild copy${full_modname})

endmacro()
