# Pastix requires METIS or METIS (partitioning and reordering tools)

if (METIS_INCLUDES AND METIS_LIBRARIES)
  set(METIS_FIND_QUIETLY TRUE)
endif (METIS_INCLUDES AND METIS_LIBRARIES)

find_path(METIS_INCLUDES 
  NAMES 
  metis.h 
  PATHS 
  $ENV{METISDIR} 
  ${INCLUDE_INSTALL_DIR} 
  PATH_SUFFIXES 
  metis
)


find_library(METIS_LIBRARIES metis PATHS $ENV{METISDIR} ${LIB_INSTALL_DIR})

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(METIS DEFAULT_MSG
                                  METIS_INCLUDES METIS_LIBRARIES)

mark_as_advanced(METIS_INCLUDES METIS_LIBRARIES)
