# -*- cmake -*-

# - Find GPerfTools (formerly Google perftools)
# Find the GPerfTools libraries
# If false, do not try to use Google perftools.
# Also defined for general use are
# - GPERFTOOLS_TCMALLOC: where to find the tcmalloc library
# - GPERFTOOLS_PROFILER: where to find the profiler library

SET(TCMALLOC_NAMES ${TCMALLOC_NAMES} tcmalloc)
find_library(GPERFTOOLS_TCMALLOC
  NAMES ${TCMALLOC_NAMES}
  PATHS /usr/lib /usr/local/lib
)
find_library(GPERFTOOLS_PROFILER
   NAMES profiler
   PATHS /usr/lib /usr/local/lib
)

IF (GPERFTOOLS_TCMALLOC AND GPERFTOOLS_PROFILER)
    SET(TCMALLOC_LIBRARIES ${GPERFTOOLS_TCMALLOC})
    SET(GPERFTOOLS_FOUND "YES")
ELSE (GPERFTOOLS_TCMALLOC AND GPERFTOOLS_PROFILER)
  SET(GPERFTOOLS_FOUND "NO")
ENDIF (GPERFTOOLS_TCMALLOC AND GPERFTOOLS_PROFILER)

IF (GPERFTOOLS_FOUND)
   MESSAGE(STATUS "Found Gperftools: ${GPERFTOOLS_PROFILER}")
ELSE (GPERFTOOLS_FOUND)
   IF (GOOGLE_PERFTOOLS_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find Google perftools library")
   ENDIF (GOOGLE_PERFTOOLS_FIND_REQUIRED)
ENDIF (GPERFTOOLS_FOUND)

MARK_AS_ADVANCED(
  GPERFTOOLS_TCMALLOC
  GPERFTOOLS_PROFILER
)

option(GTSAM_ENABLE_GPERFTOOLS                   "Enable/Disable Gperftools" OFF)
