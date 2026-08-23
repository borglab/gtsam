find_package(GeographicLib QUIET CONFIG)

if(NOT GeographicLib_FOUND)
  find_package(GeographicLib QUIET MODULE)
endif()

if(NOT GeographicLib_FOUND)
  find_path(GeographicLib_INCLUDE_DIR
    NAMES GeographicLib/Config.h
    PATH_SUFFIXES include
  )
  find_library(GeographicLib_LIBRARY
    NAMES Geographic GeographicLib
    PATH_SUFFIXES lib lib64
  )

  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(GeographicLib
    REQUIRED_VARS GeographicLib_INCLUDE_DIR GeographicLib_LIBRARY
  )

  if(GeographicLib_FOUND AND NOT TARGET GeographicLib::GeographicLib)
    add_library(GeographicLib::GeographicLib UNKNOWN IMPORTED)
    set_target_properties(GeographicLib::GeographicLib PROPERTIES
      IMPORTED_LOCATION             "${GeographicLib_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${GeographicLib_INCLUDE_DIR}"
    )
  endif()
endif()

set(GTSAM_HAVE_GEOGRAPHICLIB ${GeographicLib_FOUND})

if(TARGET GeographicLib::GeographicLib)
  message(STATUS "Using GeographicLib: ${GeographicLib_LIBRARY}")
else()
  message(STATUS "GeographicLib not found; Relevant tests disabled.")
endif()
