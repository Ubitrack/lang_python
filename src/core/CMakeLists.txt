SET(CURRENT_PACKAGE "ubitrack/core")


# SET (MODULE_SRCS_UTUTIL "wrap_utUtil.cpp" )
# add_python_module("_ututil" ${CURRENT_PACKAGE} "MODULE_SRCS_UTUTIL") 

SET (MODULE_SRCS_UTMATH "wrap_utMath.cpp")
add_python_module("_utmath" ${CURRENT_PACKAGE} "MODULE_SRCS_UTMATH") 

# SET (MODULE_SRCS_UTMEASUREMENT "wrap_utMeasurement.cpp")
# add_python_module("_utmeasurement" ${CURRENT_PACKAGE} "MODULE_SRCS_UTMEASUREMENT") 


# SET (MODULE_SRCS_UTCALIBRATION "wrap_utCalibration.cpp")
# add_python_module("_utcalibration" ${CURRENT_PACKAGE} "MODULE_SRCS_UTCALIBRATION") 

# SET (MODULE_SRCS_UTIL_SERIALIZATION "util_serialization.cpp")
# add_python_module("_serialization" ${CURRENT_PACKAGE} "MODULE_SRCS_UTIL_SERIALIZATION") 
