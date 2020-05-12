# This module will try to find METIS and define the following:
#  METIS_FOUND - True iff METIS was found
#  METIS_INCLUDE_DIRS - METIS include directories
#  METIS_LIBRARIES - METIS library paths

find_path(METIS_INCLUDE_DIR metis.h
          PATHS /usr/include /usr/local/include
          DOC "METIS include directory")

find_library(METIS_LIBRARY metis
             PATHS /usr/lib /usr/local/lib ${PROJECT_SOURCE_DIR}/metis
             DOC "Path to METIS library")

include(FindPackageHandleStandardArgs)
# handle the QUIETLY and REQUIRED arguments and set METIS_FOUND to TRUE
# if all listed variables are TRUE
find_package_handle_standard_args(METIS  DEFAULT_MSG
                                  METIS_LIBRARY METIS_INCLUDE_DIR)

mark_as_advanced(METIS_INCLUDE_DIR METIS_LIBRARY)

set(METIS_LIBRARIES ${METIS_LIBRARY})
set(METIS_INCLUDE_DIRS ${METIS_INCLUDE_DIR})
