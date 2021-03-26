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

# Configure the include directory for matlab.h
# This allows the #include to be either gtwrap/matlab.h, wrap/matlab.h or something custom.
if(NOT DEFINED GTWRAP_INCLUDE_NAME)
  set(GTWRAP_INCLUDE_NAME "gtwrap" CACHE INTERNAL "Directory name for Matlab includes")
endif()
configure_file(${PROJECT_SOURCE_DIR}/templates/matlab_wrapper.tpl.in ${PROJECT_SOURCE_DIR}/gtwrap/matlab_wrapper.tpl)

# Install CMake scripts to the standard CMake script directory.
install(FILES cmake/gtwrapConfig.cmake cmake/MatlabWrap.cmake
              cmake/PybindWrap.cmake cmake/GtwrapUtils.cmake
        DESTINATION "${SCRIPT_INSTALL_DIR}/gtwrap")

# Needed for the CMAKE_INSTALL_X variables used below.
include(GNUInstallDirs)

# Install the gtwrap python package as a directory so it can be found  by CMake
# for wrapping.
install(DIRECTORY gtwrap DESTINATION "${CMAKE_INSTALL_FULL_LIBDIR}/gtwrap")

# Install wrapping scripts as binaries to `CMAKE_INSTALL_PREFIX/bin` so they can
# be invoked for wrapping. We use DESTINATION (instead of TYPE) so we can
# support older CMake versions.
install(PROGRAMS scripts/pybind_wrap.py scripts/matlab_wrap.py
        DESTINATION ${CMAKE_INSTALL_FULL_BINDIR})

# Install pybind11 directory to `CMAKE_INSTALL_PREFIX/lib/gtwrap/pybind11` This
# will allow the gtwrapConfig.cmake file to load it later.
install(DIRECTORY pybind11 DESTINATION "${CMAKE_INSTALL_FULL_LIBDIR}/gtwrap")

# Install the matlab.h file to `CMAKE_INSTALL_PREFIX/lib/gtwrap/matlab.h`.
install(FILES matlab.h DESTINATION "${CMAKE_INSTALL_FULL_INCLUDEDIR}/gtwrap")
