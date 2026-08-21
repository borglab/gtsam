###############################################################################
# CCOLAMD and SuiteSparse_config provider

if(DEFINED GTSAM_USE_SYSTEM_CCOLAMD)
  set(_gtsam_suitesparse_selection_explicit TRUE)
else()
  set(_gtsam_suitesparse_selection_explicit FALSE)
endif()

set(_gtsam_suitesparse_option_description
  "Use system-installed CCOLAMD and SuiteSparse_config instead of the bundled copies")
option(GTSAM_USE_SYSTEM_CCOLAMD
  "${_gtsam_suitesparse_option_description}"
  ON)

if(GTSAM_USE_SYSTEM_CCOLAMD AND NOT TARGET SuiteSparse::CCOLAMD)
  find_package(CCOLAMD CONFIG QUIET)
  if(NOT TARGET SuiteSparse::CCOLAMD)
    if(_gtsam_suitesparse_selection_explicit)
      message(FATAL_ERROR
        "GTSAM_USE_SYSTEM_CCOLAMD is ON, but no CCOLAMD config package "
        "providing SuiteSparse::CCOLAMD was found")
    else()
      set(GTSAM_USE_SYSTEM_CCOLAMD OFF CACHE BOOL
        "${_gtsam_suitesparse_option_description}" FORCE)
    endif()
  endif()
endif()

if(GTSAM_USE_SYSTEM_CCOLAMD)
  if(CCOLAMD_VERSION)
    message(STATUS "Using system CCOLAMD version ${CCOLAMD_VERSION}")
  else()
    message(STATUS "Using system CCOLAMD")
  endif()
else()
  message(STATUS "Using bundled CCOLAMD and SuiteSparse_config")
endif()

# CHOLMOD is an optional numerical backend. Enable it automatically when a
# system package exports the canonical SuiteSparse target.
if(NOT TARGET SuiteSparse::CHOLMOD)
  find_package(CHOLMOD CONFIG QUIET)
endif()
if(TARGET SuiteSparse::CHOLMOD)
  set(GTSAM_ENABLE_CHOLMOD ON)
  message(STATUS "Using system CHOLMOD")
else()
  set(GTSAM_ENABLE_CHOLMOD OFF)
  message(STATUS "CHOLMOD not found; CHOLMOD linear solver disabled")
endif()

unset(_gtsam_suitesparse_option_description)
unset(_gtsam_suitesparse_selection_explicit)
