# This config file modifies CMAKE_MODULE_PATH so that the wrap cmake files may
# be included This file also allows the use of `find_package(gtwrap)` in CMake.

set(GTWRAP_DIR "${CMAKE_CURRENT_LIST_DIR}")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")

if(WIN32 AND NOT CYGWIN)
  set(SCRIPT_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/CMake")
else()
  set(SCRIPT_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib/cmake")
endif()

# Standard includes
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CMakeDependentOption)

# Load all the CMake scripts from the standard location
include(${SCRIPT_INSTALL_DIR}/gtwrap/PybindWrap.cmake)
include(${SCRIPT_INSTALL_DIR}/gtwrap/GtwrapUtils.cmake)

# Set the variables for the wrapping scripts to be used in the build.
set(PYBIND_WRAP_SCRIPT "${CMAKE_INSTALL_FULL_BINDIR}/pybind_wrap.py")
set(MATLAB_WRAP_SCRIPT "${CMAKE_INSTALL_FULL_BINDIR}/matlab_wrap.py")

# Load the pybind11 code from the library installation path
add_subdirectory(${CMAKE_INSTALL_FULL_LIBDIR}/pybind11 pybind11)
