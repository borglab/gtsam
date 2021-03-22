# This config file modifies CMAKE_MODULE_PATH so that the wrap cmake files may
# be included This file also allows the use of `find_package(gtwrap)` in CMake.

set(GTWRAP_DIR "${CMAKE_CURRENT_LIST_DIR}")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")

if(WIN32 AND NOT CYGWIN)
  set(GTWRAP_CMAKE_DIR "${GTWRAP_DIR}")
  set(GTWRAP_SCRIPT_DIR ${GTWRAP_CMAKE_DIR}/../../../bin)
  set(GTWRAP_PYTHON_PACKAGE_DIR ${GTWRAP_CMAKE_DIR}/../../../share/gtwrap)
else()
  set(GTWRAP_CMAKE_DIR "${GTWRAP_DIR}")
  set(GTWRAP_SCRIPT_DIR ${GTWRAP_CMAKE_DIR}/../../../bin)
  set(GTWRAP_PYTHON_PACKAGE_DIR ${GTWRAP_CMAKE_DIR}/../../../share/gtwrap)
endif()

# Standard includes
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)
include(CMakeDependentOption)

# Load all the CMake scripts from the standard location
include(${GTWRAP_CMAKE_DIR}/PybindWrap.cmake)
include(${GTWRAP_CMAKE_DIR}/GtwrapUtils.cmake)

# Set the variables for the wrapping scripts to be used in the build.
set(PYBIND_WRAP_SCRIPT "${GTWRAP_SCRIPT_DIR}/pybind_wrap.py")
set(MATLAB_WRAP_SCRIPT "${GTWRAP_SCRIPT_DIR}/matlab_wrap.py")

# Load the pybind11 code from the library installation path
add_subdirectory(${GTWRAP_CMAKE_DIR}/../../gtwrap/pybind11 pybind11)
