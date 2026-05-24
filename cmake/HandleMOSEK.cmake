###############################################################################
# Handle MOSEK Fusion API for SDP solving.
# MOSEK is OFF by default so normal GTSAM builds do not need MOSEK installed.

option(GTSAM_WITH_MOSEK "Build with MOSEK SDP solver support" OFF)

if(GTSAM_WITH_MOSEK)
  find_package(MOSEK)
  if(MOSEK_FOUND)
    message(STATUS "Found MOSEK: ${MOSEK_INCLUDE_DIRS}")
    message(STATUS "MOSEK libraries: ${MOSEK_LIBRARIES}")
    if(MOSEK_FUSION_LIBRARY)
      message(STATUS "MOSEK Fusion backend: prebuilt library")
    elseif(MOSEK_FUSION_SOURCES)
      message(STATUS "MOSEK Fusion backend: source files")
    endif()

    set(MOSEK_LICENSE_FILE "$ENV{MOSEKLM_LICENSE_FILE}" CACHE FILEPATH
        "Path to MOSEK license file (mosek.lic)")
    if(NOT MOSEK_LICENSE_FILE)
      message(WARNING "MOSEK_LICENSE_FILE is not set. Tests will fail unless "
          "MOSEKLM_LICENSE_FILE is set in the environment. Pass "
          "-DMOSEK_LICENSE_FILE=/path/to/mosek.lic to cmake, or set the "
          "MOSEKLM_LICENSE_FILE environment variable.")
    elseif(NOT EXISTS "${MOSEK_LICENSE_FILE}")
      message(WARNING "MOSEK_LICENSE_FILE does not exist: ${MOSEK_LICENSE_FILE}")
    endif()

    set(GTSAM_USE_MOSEK 1)
    list(APPEND GTSAM_ADDITIONAL_LIBRARIES ${MOSEK_LIBRARIES})
    list(APPEND GTSAM_ADDITIONAL_INCLUDE_DIRS ${MOSEK_INCLUDE_DIRS})
    if(MOSEK_FUSION_SOURCES)
      list(APPEND GTSAM_ADDITIONAL_SOURCES ${MOSEK_FUSION_SOURCES})
    endif()

    if(NOT WIN32)
      set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_RPATH};${MOSEK_LIBRARY_DIR}")
      set(CMAKE_BUILD_RPATH "${CMAKE_BUILD_RPATH};${MOSEK_LIBRARY_DIR}")
    endif()
  else()
    message(FATAL_ERROR "GTSAM_WITH_MOSEK is ON but MOSEK was not found. "
        "Please set MOSEK_ROOT to your MOSEK installation path.")
  endif()
else()
  set(GTSAM_USE_MOSEK 0)
endif()
