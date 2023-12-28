# ##############################################################################
# Cephes library

# For both system or bundle version, a cmake target "cephes-gtsam-if" is defined
# (interface library)

option(
  GTSAM_USE_SYSTEM_CEPHES
  "Find and use system-installed cephes. If 'off', use the one bundled with GTSAM"
  OFF)

if(GTSAM_USE_SYSTEM_CEPHES)
  # # Debian package: libmetis-dev

  # find_path(METIS_INCLUDE_DIR metis.h REQUIRED) find_library(METIS_LIBRARY
  # metis REQUIRED)

  # if(METIS_INCLUDE_DIR AND METIS_LIBRARY) mark_as_advanced(METIS_INCLUDE_DIR)
  # mark_as_advanced(METIS_LIBRARY)

  # add_library(cephes-gtsam-if INTERFACE)
  # target_include_directories(cephes-gtsam-if BEFORE INTERFACE
  # ${METIS_INCLUDE_DIR} # gtsam_unstable/partition/FindSeparator-inl.h uses
  # internal metislib.h API # via extern "C"
  # $<BUILD_INTERFACE:${GTSAM_SOURCE_DIR}/gtsam/3rdparty/metis/libmetis>
  # $<BUILD_INTERFACE:${GTSAM_SOURCE_DIR}/gtsam/3rdparty/metis/GKlib> )
  # target_link_libraries(cephes-gtsam-if INTERFACE ${METIS_LIBRARY}) endif()

else()
  # Bundled version:
  add_subdirectory(${GTSAM_SOURCE_DIR}/gtsam/3rdparty/cephes)

  list(APPEND GTSAM_EXPORTED_TARGETS cephes-gtsam)
  set(GTSAM_EXPORTED_TARGETS
      "${GTSAM_EXPORTED_TARGETS}"
      PARENT_SCOPE)

  add_library(cephes-gtsam-if INTERFACE)
  target_link_libraries(cephes-gtsam-if INTERFACE cephes-gtsam)

endif()

list(APPEND GTSAM_EXPORTED_TARGETS cephes-gtsam-if)
install(
  TARGETS cephes-gtsam-if
  EXPORT GTSAM-exports
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
