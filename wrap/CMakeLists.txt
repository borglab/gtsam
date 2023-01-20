cmake_minimum_required(VERSION 3.9)

# Set the project name and version
project(gtwrap VERSION 1.0)

# ##############################################################################
# General configuration

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/GtwrapUtils.cmake)
gtwrap_get_python_version(${WRAP_PYTHON_VERSION})

# Set the variables to be used for the cmake config file.
if(WIN32 AND NOT CYGWIN)
  set(INSTALL_CMAKE_DIR CMake/${PROJECT_NAME})
else()
  set(INSTALL_CMAKE_DIR lib/cmake/${PROJECT_NAME})
endif()

set(INSTALL_LIB_DIR lib/${PROJECT_NAME})
set(INSTALL_BIN_DIR bin/${PROJECT_NAME})
set(INSTALL_INCLUDE_DIR include/${PROJECT_NAME})

# ##############################################################################
# Package Configuration

# Helper functions for generating the gtwrapConfig.cmake file correctly.
include(CMakePackageConfigHelpers)

# Configure the config file which is used for `find_package`.
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/gtwrapConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/cmake/gtwrapConfig.cmake
  INSTALL_DESTINATION "${INSTALL_CMAKE_DIR}"
  PATH_VARS INSTALL_CMAKE_DIR INSTALL_LIB_DIR INSTALL_BIN_DIR
            INSTALL_INCLUDE_DIR
  INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})

# Set all the install paths
set(GTWRAP_CMAKE_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/${INSTALL_CMAKE_DIR})
set(GTWRAP_LIB_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/${INSTALL_LIB_DIR})
set(GTWRAP_BIN_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/${INSTALL_BIN_DIR})
set(GTWRAP_INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_PREFIX}/${INSTALL_INCLUDE_DIR})

# ##############################################################################
# Install the package

# Install CMake scripts to the standard CMake script directory.
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cmake/gtwrapConfig.cmake
              cmake/MatlabWrap.cmake cmake/PybindWrap.cmake
              cmake/GtwrapUtils.cmake DESTINATION "${GTWRAP_CMAKE_INSTALL_DIR}")

# Configure the include directory for matlab.h This allows the #include to be
# either gtwrap/matlab.h, wrap/matlab.h or something custom.
if(NOT DEFINED GTWRAP_INCLUDE_NAME)
  set(GTWRAP_INCLUDE_NAME
      "gtwrap"
      CACHE INTERNAL "Directory name for Matlab includes")
endif()

configure_file(${PROJECT_SOURCE_DIR}/templates/matlab_wrapper.tpl.in
               ${PROJECT_SOURCE_DIR}/gtwrap/matlab_wrapper/matlab_wrapper.tpl)

# Install the gtwrap python package as a directory so it can be found  by CMake
# for wrapping.
install(DIRECTORY gtwrap DESTINATION "${GTWRAP_LIB_INSTALL_DIR}")

# Install pybind11 directory to `CMAKE_INSTALL_PREFIX/lib/gtwrap/pybind11` This
# will allow the gtwrapConfig.cmake file to load it later.
install(DIRECTORY pybind11 DESTINATION "${GTWRAP_LIB_INSTALL_DIR}")

# Install wrapping scripts as binaries to `CMAKE_INSTALL_PREFIX/bin` so they can
# be invoked for wrapping. We use DESTINATION (instead of TYPE) so we can
# support older CMake versions.
install(PROGRAMS scripts/pybind_wrap.py scripts/matlab_wrap.py
        DESTINATION "${GTWRAP_BIN_INSTALL_DIR}")

# Install the matlab.h file to `CMAKE_INSTALL_PREFIX/lib/gtwrap/matlab.h`.
install(FILES matlab.h DESTINATION "${GTWRAP_INCLUDE_INSTALL_DIR}")

string(ASCII 27 Esc)
set(gtwrap "${Esc}[1;36mgtwrap${Esc}[m")
message(STATUS "${gtwrap} Package config : ${GTWRAP_CMAKE_INSTALL_DIR}")
message(STATUS "${gtwrap} version        : ${PROJECT_VERSION}")
message(STATUS "${gtwrap} CMake path     : ${GTWRAP_CMAKE_INSTALL_DIR}")
message(STATUS "${gtwrap} library path   : ${GTWRAP_LIB_INSTALL_DIR}")
message(STATUS "${gtwrap} binary path    : ${GTWRAP_BIN_INSTALL_DIR}")
message(STATUS "${gtwrap} header path    : ${GTWRAP_INCLUDE_INSTALL_DIR}")
