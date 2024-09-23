# ##############################################################################
# Cephes library

# For both system or bundle version, a cmake target "cephes-gtsam-if" is defined
# (interface library)


add_subdirectory(${GTSAM_SOURCE_DIR}/gtsam/3rdparty/cephes)

list(APPEND GTSAM_EXPORTED_TARGETS cephes-gtsam)

add_library(cephes-gtsam-if INTERFACE)
target_link_libraries(cephes-gtsam-if INTERFACE cephes-gtsam)

list(APPEND GTSAM_EXPORTED_TARGETS cephes-gtsam-if)
install(
  TARGETS cephes-gtsam-if
  EXPORT GTSAM-exports
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
