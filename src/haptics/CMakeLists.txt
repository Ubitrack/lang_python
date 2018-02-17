SET(CURRENT_PACKAGE "ubitrack/haptics")

SET (MODULE_SRCS_UTHAPTICS "wrap_utHaptics.cpp")
add_python_module("_uthaptics" ${CURRENT_PACKAGE} "MODULE_SRCS_UTHAPTICS")