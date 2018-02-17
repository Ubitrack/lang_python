SET(CURRENT_PACKAGE "ubitrack/facade")

SET (MODULE_SRCS_UTFACADE "wrap_utFacade.cpp")
add_python_module("_utfacade" ${CURRENT_PACKAGE} "MODULE_SRCS_UTFACADE") 