cmake_minimum_required(VERSION 3.9)

# Set the project name and version
project(GTwrap VERSION 1.0)

# ##############################################################################
# General configuration

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

# Install scripts to the standard CMake script directory.
install(FILES cmake/gtwrapConfig.cmake cmake/PybindWrap.cmake
              cmake/GtwrapUtils.cmake
        DESTINATION "${SCRIPT_INSTALL_DIR}/gtwrap")

# Install wrapping scripts as binaries to `CMAKE_INSTALL_PREFIX/bin` so they can
# be invoked for wrapping.
install(PROGRAMS scripts/pybind_wrap.py scripts/matlab_wrap.py TYPE BIN)

# Install pybind11 directory to `CMAKE_INSTALL_PREFIX/lib/pybind11` This will
# allow the gtwrapConfig.cmake file to load it later.
install(DIRECTORY pybind11 TYPE LIB)

# ##############################################################################
# Install the Python package
find_package(
  Python ${WRAP_PYTHON_VERSION}
  COMPONENTS Interpreter
  REQUIRED)

#TODO add correct flags for virtualenv

# Finally install the gtwrap python package.
execute_process(COMMAND ${Python_EXECUTABLE} setup.py install
                WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
