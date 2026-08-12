cmake_minimum_required(VERSION 3.10)
enable_testing()
project(
  cephes
  DESCRIPTION "Cephes Mathematical Function Library"
  VERSION 1.0.0
  LANGUAGES C)

set(CEPHES_HEADER_FILES
    cephes.h
    cephes/igam.h
    cephes/lanczos.h
    cephes/mconf.h
    cephes/polevl.h
    cephes/sf_error.h)

# Add header files
install(FILES ${CEPHES_HEADER_FILES} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/3rdparty/cephes)

set(CEPHES_SOURCES
    cephes/const.c
    cephes/gamma.c
    cephes/igam.c
    cephes/igami.c
    cephes/lanczos.c
    cephes/sf_error.c
    cephes/unity.c
    cephes/zeta.c)

# Add library source files
add_library(cephes-gtsam ${GTSAM_LIBRARY_TYPE} ${CEPHES_SOURCES})

# cephes/gamma.c, igam.c, etc. call libm functions (e.g. sin) directly, but
# nothing here previously linked cephes-gtsam against libm, leaving it to be
# resolved transitively through whatever else the final binary happens to
# link. That can crash the dynamic linker while relinking an IFUNC-resolved
# symbol (e.g. sin) depending on load order; see e.g.
# https://bugzilla.redhat.com/show_bug.cgi?id=1883501 for the same failure
# mode in an unrelated project. find_library naturally no-ops on platforms
# without a separate libm (Windows, and effectively macOS where it's part of
# libSystem already).
find_library(MATH_LIBRARY m)
if(MATH_LIBRARY)
  target_link_libraries(cephes-gtsam PUBLIC ${MATH_LIBRARY})
endif()

# Add include directory (aka headers)
target_include_directories(
  cephes-gtsam BEFORE PUBLIC $<INSTALL_INTERFACE:include/gtsam/3rdparty/cephes/>
                             $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

set_target_properties(
  cephes-gtsam
  PROPERTIES VERSION ${PROJECT_VERSION}
             SOVERSION ${PROJECT_VERSION_MAJOR}
             C_STANDARD 99
             WINDOWS_EXPORT_ALL_SYMBOLS ON)

if(WIN32)
  set_target_properties(cephes-gtsam PROPERTIES COMPILE_FLAGS /w)
endif()

if(APPLE)
  set_target_properties(cephes-gtsam PROPERTIES INSTALL_NAME_DIR
                                                "${CMAKE_INSTALL_PREFIX}/lib")
endif()

install(
  TARGETS cephes-gtsam
  EXPORT GTSAM-exports
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
