cmake_minimum_required(VERSION 3.10)

# Set the project name and version
project(GTwrap VERSION 1.0)

# ##############################################################################
# General configuration

# Sets the path to the interface parser.
set(GTWRAP_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

set(WRAP_PYTHON_VERSION
    "Default"
    CACHE STRING "The Python version to use for wrapping")

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/GtwrapUtils.cmake)
gtwrap_get_python_version(${WRAP_PYTHON_VERSION})

# ##############################################################################
# Install the CMake file to be used by other projects
if(WIN32 AND NOT CYGWIN)
  set(SCRIPT_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/CMake")
else()
  set(SCRIPT_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib/cmake")
endif()

configure_file(cmake/PybindWrap.cmake.in
               ${CMAKE_CURRENT_SOURCE_DIR}/cmake/PybindWrap.cmake @ONLY)
# Install scripts
install(FILES cmake/PybindWrap.cmake cmake/gtwrapConfig.cmake
              cmake/GtwrapUtils.cmake
        DESTINATION "${SCRIPT_INSTALL_DIR}/gtwrap")

# ##############################################################################
# Install the Python package
find_package(
  Python ${WRAP_PYTHON_VERSION}
  COMPONENTS Interpreter
  REQUIRED)

# Detect virtualenv and set Pip args accordingly
# https://www.scivision.dev/cmake-install-python-package/
if(DEFINED ENV{VIRTUAL_ENV} OR DEFINED ENV{CONDA_PREFIX})
  set(_pip_args)
else()
  set(_pip_args "--user")
endif()

# We install in development mode (-e flag) so any updates to the package are
# automatically propagated.
execute_process(COMMAND ${Python_EXECUTABLE} -m pip install -e . ${_pip_args}
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
