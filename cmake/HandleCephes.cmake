# ##############################################################################
# Cephes library

# For both system or bundle version, a cmake target "cephes-gtsam-if" is defined
# (interface library)


add_subdirectory(${GTSAM_SOURCE_DIR}/gtsam/3rdparty/cephes)

# Match the bundled headers' install destination without changing vendor code.
set_property(TARGET cephes-gtsam PROPERTY INTERFACE_INCLUDE_DIRECTORIES
  $<BUILD_INTERFACE:${GTSAM_SOURCE_DIR}/gtsam/3rdparty/cephes>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/gtsam/3rdparty/cephes/>)

# The bundled CMakeLists links the absolute path returned by
# find_library(MATH_LIBRARY m). That path is exported as is, so consumers of
# GTSAMConfig.cmake try to link a libm.so that only exists on the build machine
# (e.g. the sysroot of a conda build environment). Export libm by name instead.
if(MATH_LIBRARY)
  set_property(TARGET cephes-gtsam PROPERTY INTERFACE_LINK_LIBRARIES m)
endif()

list(APPEND GTSAM_EXPORTED_TARGETS cephes-gtsam)

add_library(cephes-gtsam-if INTERFACE)
target_link_libraries(cephes-gtsam-if INTERFACE cephes-gtsam)

list(APPEND GTSAM_EXPORTED_TARGETS cephes-gtsam-if)
install(
  TARGETS cephes-gtsam-if
  EXPORT GTSAM-exports
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
