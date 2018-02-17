SET(CURRENT_PACKAGE "ubitrack/dataflow")

SET (MODULE_SRCS_UTDATAFLOW "wrap_utDataflow.cpp")
add_python_module("_utdataflow" ${CURRENT_PACKAGE} "MODULE_SRCS_UTDATAFLOW") 


SET (MODULE_SRCS_UTGRAPH "wrap_utGraph.cpp")
add_python_module("_utgraph" ${CURRENT_PACKAGE} "MODULE_SRCS_UTGRAPH") 
