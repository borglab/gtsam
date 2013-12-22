# This is FindCppUnitLite.cmake
# DEPRECIATED: Use config file approach to pull in targets from gtsam
# CMake module to locate the CppUnit package
# The following variables will be defined:
#
# CppUnitLite_FOUND          : TRUE if the package has been successfully found
# CppUnitLite_INCLUDE_DIR    : paths to CppUnitLite's INCLUDE directories
# CppUnitLite_LIBS           : paths to CppUnitLite's libraries

# If gtsam was found by a previous call to FindGTSAM, prefer to find the
# CppUnitLite in the same place as that gtsam
if(GTSAM_LIBS)
  # Find the gtsam library path found by a previous call to FindGTSAM
  get_filename_component(_gtsam_lib_dir "${GTSAM_LIBS}" PATH)
  get_filename_component(_gtsam_lib_dir_name "${_gtsam_lib_dir}" NAME)
  # If the gtsam library was in a directory called 'gtsam', it means we found
  # gtsam in the source tree, otherwise (probably 'lib') in an installed location.
  if(_gtsam_lib_dir_name STREQUAL "gtsam")
    get_filename_component(_gtsam_build_dir "${_gtsam_lib_dir}" PATH)
    set(_gtsam_cppunitlite_dir "${_gtsam_build_dir}/CppUnitLite")
  else()
    set(_gtsam_cppunitlite_dir "${_gtsam_lib_dir}")
  endif()
endif()

if(GTSAM_LIBS)
  # Twice to get the build directory, not build/gtsam
  get_filename_component(_gtsam_build_dir "${GTSAM_LIBS}" PATH)
  get_filename_component(_gtsam_build_dir "${_gtsam_build_dir}" PATH)
endif()

# Find include dirs
find_path(CppUnitLite_INCLUDE_DIR CppUnitLite/Test.h
    PATHS "${GTSAM_INCLUDE_DIR}" ${CMAKE_INSTALL_PREFIX}/include "$ENV{HOME}/include" /usr/local/include /usr/include
    DOC "CppUnitLite INCLUDE directories")

# Find libraries
find_library(CppUnitLite_LIBS NAMES CppUnitLite
  HINTS "${_gtsam_cppunitlite_dir}" ${CMAKE_INSTALL_PREFIX}/lib "$ENV{HOME}/lib" /usr/local/lib /usr/lib
  DOC "CppUnitLite libraries")

# handle the QUIETLY and REQUIRED arguments and set CppUnitLite_FOUND to TRUE
# if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(CppUnitLite DEFAULT_MSG
                                  CppUnitLite_LIBS CppUnitLite_INCLUDE_DIR)

