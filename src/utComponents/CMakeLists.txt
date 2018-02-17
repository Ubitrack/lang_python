# a python scriptable ubitrack node

set(the_component "PythonComponent")
set(the_description "The Python Scribtable UbiTrack component")

# create option to enable/disable this component
option(BUILD_${the_component} "Include ${the_component} component into the UbiTrack python build" ON})

# needs more ENV{} magic ;)
set(UBITRACK_INSTALL_ROOT "/usr/local")

set(HEADERS "")
set(SOURCES "PythonComponent")

if(WIN32)
  set(UBITRACK_DEBUG_POSTFIX d)
else()
  set(UBITRACK_DEBUG_POSTFIX "")
endif()

set(LIBRARY_OUTPUT_PATH         "${CMAKE_BINARY_DIR}/lib")
set(UBITRACK_LIB_INSTALL_PATH     lib${LIB_SUFFIX})
set(UBITRACK_INCLUDE_INSTALL_PATH include)

math(EXPR SIZEOF_VOID_P_BITS "8 * ${CMAKE_SIZEOF_VOID_P}")
if(LIB_SUFFIX AND NOT SIZEOF_VOID_P_BITS EQUAL LIB_SUFFIX)
set(UBITRACK_CONFIG_INSTALL_PATH lib${LIB_SUFFIX}/cmake/ubitrack)
else()
set(UBITRACK_CONFIG_INSTALL_PATH share/UbiTrack)
endif()


set(UBITRACK_COMPONENT_INSTALL_DIRECTORY "ubitrack")
set(UBITRACK_COMPONENT_INSTALL_PATH "${UBITRACK_LIB_INSTALL_PATH}/${UBITRACK_COMPONENT_INSTALL_DIRECTORY}")
set(UBITRACK_COMPONENT_BIN_INSTALL_PATH "bin/${UBITRACK_COMPONENT_INSTALL_DIRECTORY}")

if (${BUILD_${the_component}})

	project(${the_component})
	add_library(${the_component} SHARED ${HEADERS} ${SOURCES})


	target_link_libraries(${the_component} ${MODULE_LIBS})

	set_target_properties(${the_component} PROPERTIES
		OUTPUT_NAME "${the_component}"
		DEBUG_POSTFIX "${UBITRACK_DEBUG_POSTFIX}"
		ARCHIVE_OUTPUT_DIRECTORY ${UBITRACK_COMPONENT_INSTALL_PATH}
		LIBRARY_OUTPUT_DIRECTORY ${UBITRACK_COMPONENT_INSTALL_PATH}
		RUNTIME_OUTPUT_DIRECTORY ${UBITRACK_COMPONENT_BIN_INSTALL_PATH}
		INSTALL_NAME_DIR lib
	)

	if(MSVC)
	  set_target_properties(${the_component} PROPERTIES COMPILE_FLAGS "${UBITRACK_COMPILE_FLAGS}")
	  set_target_properties(${the_component} PROPERTIES LINK_FLAGS "${UBITRACK_LINK_FLAGS}")
	endif()

	if (NOT Boost_USE_STATIC_LIBS)
		if(MSVC)
			# force dynamic linking of boost libs on windows ..
			set(COMPILE_DEFINITIONS $COMPILE_DEFINITIONS} "BOOST_ALL_DYN_LINK")
		endif(MSVC)
	endif (NOT Boost_USE_STATIC_LIBS)

	set_target_properties(${the_component} PROPERTIES COMPILE_DEFINITIONS "${COMPILE_DEFINITIONS}")

	if(MSVC)
		if(CMAKE_CROSSCOMPILING)
			set_target_properties(${the_component} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:secchk")
		endif()
		set_target_properties(${the_component} PROPERTIES LINK_FLAGS "/NODEFAULTLIB:libc /DEBUG")
	endif()

	install(TARGETS ${the_component}
		RUNTIME DESTINATION ${UBITRACK_COMPONENT_BIN_INSTALL_PATH} COMPONENT main
		LIBRARY DESTINATION ${UBITRACK_COMPONENT_INSTALL_PATH} COMPONENT main
		ARCHIVE DESTINATION ${UBITRACK_COMPONENT_INSTALL_PATH} COMPONENT main
	)
endif (${BUILD_${the_component}})
