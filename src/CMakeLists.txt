set(MODULE_SOURCES
	ubitrack.cpp
	BackgroundImage.cpp
	)

set(MODULE_HEADERS
		core_math.hpp
		core_calibration.hpp
		core_measurement.hpp
		core_serialization.hpp
		core_util.hpp
		dataflow_dataflow.hpp
		dataflow_graph.hpp
        hapticcalibration_main.hpp
		facade_main.hpp
		vision_main.hpp
		visualization_main.hpp
		)


pybind11_add_module(ubitrack MODULE ${MODULE_SOURCES} ${MODULE_HEADERS})
target_link_libraries( ubitrack PRIVATE ${MODULE_LIBS})

if(WIN32)
    if(MSVC)
        set(UBITRACK_COMPILE_FLAGS ${UBITRACK_COMPILE_FLAGS} "/bigobj ")
    endif(MSVC)
endif(WIN32)

foreach(_flag ${UBITRACK_COMPILE_FLAGS})
	set_target_properties(ubitrack PROPERTIES COMPILE_FLAGS "${_flag}")
endforeach()
foreach(_flag ${UBITRACK_LINK_FLAGS})
	set_target_properties(ubitrack PROPERTIES LINK_FLAGS "${_flag}")
endforeach()
foreach(_flag ${UBITRACK_LINK_FLAGS_DEBUG})
	set_target_properties(ubitrack PROPERTIES LINK_FLAGS_DEBUG "${_flag}")
endforeach()
foreach(_symb ${UBITRACK_DEFINES})
	set_target_properties(ubitrack PROPERTIES DEFINE_SYMBOL ${_symb})
endforeach()

# set compiler Definitions
set_target_properties(ubitrack PROPERTIES COMPILE_DEFINITIONS "${UBITRACK_COMPILE_DEFINITIONS}")

install(TARGETS ubitrack
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/python
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/python
  )
