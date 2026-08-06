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
  find_package(METIS CONFIG NAMES metis)
  if(METIS_FOUND)
    set(GTSAM_METIS_LINK_TARGET metis)
  else()
    find_path(METIS_INCLUDE_DIR metis.h REQUIRED)
    find_library(METIS_LIBRARY metis REQUIRED)
    if(METIS_INCLUDE_DIR AND METIS_LIBRARY)
      set(METIS_FOUND ON)
      set(GTSAM_METIS_LINK_TARGET ${METIS_LIBRARY})
    endif()
  endif()

  if(METIS_FOUND)
    mark_as_advanced(METIS_INCLUDE_DIR)
    mark_as_advanced(METIS_LIBRARY)
    add_library(metis-gtsam-if INTERFACE)
    target_include_directories(metis-gtsam-if BEFORE INTERFACE ${METIS_INCLUDE_DIR})
    target_link_libraries(metis-gtsam-if INTERFACE ${GTSAM_METIS_LINK_TARGET})
  endif()
else()
  # Bundled version:
  option(GTSAM_BUILD_METIS_EXECUTABLES "Build metis library executables" OFF)
  add_subdirectory(${GTSAM_SOURCE_DIR}/gtsam/3rdparty/metis)

  target_include_directories(metis-gtsam BEFORE PUBLIC
    $<INSTALL_INTERFACE:include/gtsam/3rdparty/metis/>
    $<BUILD_INTERFACE:${GTSAM_SOURCE_DIR}/gtsam/3rdparty/metis/include>
  )

  add_library(metis-gtsam-if INTERFACE)
  target_link_libraries(metis-gtsam-if INTERFACE metis-gtsam)
endif()

list(APPEND GTSAM_EXPORTED_TARGETS metis-gtsam-if)
install(TARGETS metis-gtsam-if EXPORT GTSAM-exports ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
