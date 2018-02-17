SET(CURRENT_PACKAGE "ubitrack/visualization")

SET (MODULE_SRCS_UTVISUALIZATION "wrap_utVisualization.cpp"
		"BackgroundImage.cpp")
				 
add_python_module("_utvisualization" ${CURRENT_PACKAGE} "MODULE_SRCS_UTVISUALIZATION")
