###############################################################################
# Metis library

# For both system or bundle version, a cmake target "metis-gtsam-if" is defined (interface library)

# Dont try to use metis if GTSAM_SUPPORT_NESTED_DISSECTION is disabled:
if (NOT GTSAM_SUPPORT_NESTED_DISSECTION)
  return()
endif()

option(GTSAM_USE_SYSTEM_METIS "Find and use system-installed libmetis. If 'off', use the one bundled with GTSAM" OFF)

if(GTSAM_USE_SYSTEM_METIS)
  # Debian package: libmetis-dev

  find_path(METIS_INCLUDE_DIR metis.h REQUIRED)
  find_library(METIS_LIBRARY metis REQUIRED)

  if(METIS_INCLUDE_DIR AND METIS_LIBRARY)
    mark_as_advanced(METIS_INCLUDE_DIR)
    mark_as_advanced(METIS_LIBRARY)

    add_library(metis-gtsam-if INTERFACE)
    target_include_directories(metis-gtsam-if BEFORE INTERFACE ${METIS_INCLUDE_DIR})
    target_link_libraries(metis-gtsam-if INTERFACE ${METIS_LIBRARY})
  endif()
else()
  # Bundled version:
  option(GTSAM_BUILD_METIS_EXECUTABLES "Build metis library executables" OFF)
  add_subdirectory(${GTSAM_SOURCE_DIR}/gtsam/3rdparty/metis)

  target_include_directories(metis-gtsam BEFORE PUBLIC
    $<BUILD_INTERFACE:${GTSAM_SOURCE_DIR}/gtsam/3rdparty/metis/include>
    $<BUILD_INTERFACE:${GTSAM_SOURCE_DIR}/gtsam/3rdparty/metis/libmetis>
    $<BUILD_INTERFACE:${GTSAM_SOURCE_DIR}/gtsam/3rdparty/metis/GKlib>
    $<INSTALL_INTERFACE:include/gtsam/3rdparty/metis/>
  )

  add_library(metis-gtsam-if INTERFACE)
  target_link_libraries(metis-gtsam-if INTERFACE metis-gtsam)
endif()

list(APPEND GTSAM_EXPORTED_TARGETS metis-gtsam-if)
install(TARGETS metis-gtsam-if EXPORT GTSAM-exports ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
