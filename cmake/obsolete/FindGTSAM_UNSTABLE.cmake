# This is FindGTSAM_UNSTABLE.cmake
# DEPRECIATED: Use config file approach to pull in targets from gtsam
# CMake module to locate the GTSAM_UNSTABLE package
#
# The following cache variables may be set before calling this script:
#
# GTSAM_UNSTABLE_DIR (or GTSAM_UNSTABLE_ROOT): (Optional) The install prefix OR source tree of gtsam_unstable (e.g. /usr/local or src/gtsam_unstable)
# GTSAM_UNSTABLE_BUILD_NAME:           (Optional) If compiling against a source tree, the name of the build directory
#                              within it (e.g build-debug).  Without this defined, this script tries to
#                              intelligently find the build directory based on the project's build directory name
#                              or based on the build type (Debug/Release/etc).
#
# The following variables will be defined:
#
# GTSAM_UNSTABLE_FOUND          : TRUE if the package has been successfully found
# GTSAM_UNSTABLE_INCLUDE_DIR    : paths to GTSAM_UNSTABLE's INCLUDE directories
# GTSAM_UNSTABLE_LIBS           : paths to GTSAM_UNSTABLE's libraries
#
# NOTES on compiling against an uninstalled GTSAM_UNSTABLE build tree:
# - A GTSAM_UNSTABLE source tree will be automatically searched for in the directory
#   'gtsam_unstable' next to your project directory, after searching
#   CMAKE_INSTALL_PREFIX and $HOME, but before searching /usr/local and /usr.
# - The build directory will be searched first with the same name as your
#   project's build directory, e.g. if you build from 'MyProject/build-optimized',
#   'gtsam_unstable/build-optimized' will be searched first.  Next, a build directory for
#   your project's build type, e.g. if CMAKE_BUILD_TYPE in your project is
#   'Release', then 'gtsam_unstable/build-release' will be searched next.  Finally, plain
#   'gtsam_unstable/build' will be searched.
# - You can control the gtsam build directory name directly by defining the CMake
#   cache variable 'GTSAM_UNSTABLE_BUILD_NAME', then only 'gtsam/${GTSAM_UNSTABLE_BUILD_NAME} will
#   be searched.
# - Use the standard CMAKE_PREFIX_PATH, or GTSAM_UNSTABLE_DIR, to find a specific gtsam
#   directory.

# Get path suffixes to help look for gtsam_unstable
if(GTSAM_UNSTABLE_BUILD_NAME)
  set(gtsam_unstable_build_names "${GTSAM_UNSTABLE_BUILD_NAME}/gtsam_unstable")
else()
  # lowercase build type
  string(TOLOWER "${CMAKE_BUILD_TYPE}" build_type_suffix)
  # build suffix of this project
  get_filename_component(my_build_name "${CMAKE_BINARY_DIR}" NAME)
  
  set(gtsam_unstable_build_names "${my_build_name}/gtsam_unstable" "build-${build_type_suffix}/gtsam_unstable" "build/gtsam_unstable")
endif()

# Use GTSAM_UNSTABLE_ROOT or GTSAM_UNSTABLE_DIR equivalently
if(GTSAM_UNSTABLE_ROOT AND NOT GTSAM_UNSTABLE_DIR)
  set(GTSAM_UNSTABLE_DIR "${GTSAM_UNSTABLE_ROOT}")
endif()

if(GTSAM_UNSTABLE_DIR)
  # Find include dirs
  find_path(GTSAM_UNSTABLE_INCLUDE_DIR gtsam_unstable/base/DSF.h
    PATHS "${GTSAM_UNSTABLE_DIR}/include" "${GTSAM_UNSTABLE_DIR}" NO_DEFAULT_PATH
    DOC "GTSAM_UNSTABLE include directories")

  # Find libraries
  find_library(GTSAM_UNSTABLE_LIBS NAMES gtsam_unstable
    HINTS "${GTSAM_UNSTABLE_DIR}/lib" "${GTSAM_UNSTABLE_DIR}" NO_DEFAULT_PATH
    PATH_SUFFIXES ${gtsam_unstable_build_names}
    DOC "GTSAM_UNSTABLE libraries")
else()
  # Find include dirs
  set(extra_include_paths ${CMAKE_INSTALL_PREFIX}/include "$ENV{HOME}/include" "${PROJECT_SOURCE_DIR}/../gtsam" /usr/local/include /usr/include)
  find_path(GTSAM_UNSTABLE_INCLUDE_DIR gtsam_unstable/base/DSF.h
    PATHS ${extra_include_paths}
    DOC "GTSAM_UNSTABLE include directories")
  if(NOT GTSAM_UNSTABLE_INCLUDE_DIR)
    message(STATUS "Searched for gtsam_unstable headers in default paths plus ${extra_include_paths}")
  endif()

  # Find libraries
  find_library(GTSAM_UNSTABLE_LIBS NAMES gtsam_unstable
    HINTS ${CMAKE_INSTALL_PREFIX}/lib "$ENV{HOME}/lib" "${PROJECT_SOURCE_DIR}/../gtsam" /usr/local/lib /usr/lib
    PATH_SUFFIXES ${gtsam_unstable_build_names}
    DOC "GTSAM_UNSTABLE libraries")
endif()

# handle the QUIETLY and REQUIRED arguments and set GTSAM_UNSTABLE_FOUND to TRUE
# if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GTSAM_UNSTABLE DEFAULT_MSG
                                  GTSAM_UNSTABLE_LIBS GTSAM_UNSTABLE_INCLUDE_DIR)
 



