cmake_minimum_required(VERSION 3.9...3.29)
if (POLICY CMP0167)
cmake_policy(SET CMP0167 OLD) # Don't complain about boost
endif()

# Set the version number for the library
set (GTSAM_VERSION_MAJOR 4)
set (GTSAM_VERSION_MINOR 3)
set (GTSAM_VERSION_PATCH 0)
set (GTSAM_PRERELEASE_VERSION "a1")
math (EXPR GTSAM_VERSION_NUMERIC "10000 * ${GTSAM_VERSION_MAJOR} + 100 * ${GTSAM_VERSION_MINOR} + ${GTSAM_VERSION_PATCH}")

# Set the version string for the library. 
#
# If the environment variable DEVELOP is set, then the version string will be
# "MAJOR.MINORprerelease.devTIMESTAMP". TIMESTAMP is another environment variable that should be set to the current
# datetime. See build-cibw.yaml for example usage.
#
# If the prerelease version is empty, then the version string will be "MAJOR.MINOR.PATCH". Otherwise, the version
# string will be "MAJOR.MINORprerelease".
if (DEFINED ENV{DEVELOP})
    set (GTSAM_VERSION_STRING "${GTSAM_VERSION_MAJOR}.${GTSAM_VERSION_MINOR}${GTSAM_PRERELEASE_VERSION}.dev$ENV{TIMESTAMP}")
    set (SETUP_NAME "gtsam-develop")
elseif ("${GTSAM_PRERELEASE_VERSION}" STREQUAL "")
    set (GTSAM_VERSION_STRING "${GTSAM_VERSION_MAJOR}.${GTSAM_VERSION_MINOR}.${GTSAM_VERSION_PATCH}")
    set (SETUP_NAME "gtsam")
else()
    set (GTSAM_VERSION_STRING "${GTSAM_VERSION_MAJOR}.${GTSAM_VERSION_MINOR}${GTSAM_PRERELEASE_VERSION}")
    set (SETUP_NAME "gtsam")
endif()

project(GTSAM
        LANGUAGES CXX C
        VERSION "${GTSAM_VERSION_MAJOR}.${GTSAM_VERSION_MINOR}.${GTSAM_VERSION_PATCH}")

set (CMAKE_PROJECT_VERSION_MAJOR ${GTSAM_VERSION_MAJOR})
set (CMAKE_PROJECT_VERSION_MINOR ${GTSAM_VERSION_MINOR})
set (CMAKE_PROJECT_VERSION_PATCH ${GTSAM_VERSION_PATCH})

###############################################################################
# Gather information, perform checks, set defaults

# Option to enable AddressSanitizer
option(GTSAM_ENABLE_ASAN "Enable AddressSanitizer for memory error detection" OFF)

if (GTSAM_ENABLE_ASAN)
    message(STATUS "Enabling AddressSanitizer (ASan)")
    if (MSVC)
        # MSVC-specific ASan flags
        # /fsanitize=address enables ASan
        # /Zi is for debug information (PDB files)
        # /Od is for disabling optimization (Debug builds)
        # /MD is for linking with the DLL version of the C runtime library
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /fsanitize=address /Zi /Od")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /fsanitize=address /Zi /Od")

        # Linker flags for MSVC ASan
        # /fsanitize=address enables ASan linking
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /fsanitize=address")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} /fsanitize=address")
        set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} /fsanitize=address")
    else()
        # Clang/GCC ASan flags (already there)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer")
        set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=address -fno-omit-frame-pointer")

        # Linker flags
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fsanitize=address")
        set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -fsanitize=address")
        set(CMAKE_MODULE_LINKER_FLAGS "${CMAKE_MODULE_LINKER_FLAGS} -fsanitize=address")
    endif()
endif()

if(MSVC)
  set(MSVC_LINKER_FLAGS "/FORCE:MULTIPLE")
  set(CMAKE_EXE_LINKER_FLAGS ${MSVC_LINKER_FLAGS})
  set(CMAKE_MODULE_LINKER_FLAGS ${MSVC_LINKER_FLAGS})
  set(CMAKE_SHARED_LINKER_FLAGS ${MSVC_LINKER_FLAGS})
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(GtsamMakeConfigFile)
include(GNUInstallDirs)

# guard against in-source builds
if(${GTSAM_SOURCE_DIR} STREQUAL ${GTSAM_BINARY_DIR})
  message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. ")
endif()

include(cmake/HandleGeneralOptions.cmake)   # CMake build options

# Load build type flags and default to Debug mode
include(GtsamBuildTypes)

# Use macros for creating tests/timing scripts
include(GtsamTesting)
include(GtsamPrinting)

############### Decide on BOOST ######################################
# Enable or disable serialization with GTSAM_ENABLE_BOOST_SERIALIZATION
option(GTSAM_ENABLE_BOOST_SERIALIZATION "Enable Boost serialization" ON)
option(GTSAM_USE_BOOST_FEATURES "Enable Features that use Boost" ON)

if(GTSAM_ENABLE_BOOST_SERIALIZATION OR GTSAM_USE_BOOST_FEATURES)
include(cmake/HandleBoost.cmake)
endif()
######################################################################

# ############## Decide on GeographicLib ######################################
# Enable or disable GeographicLib integration with GTSAM_ENABLE_GEOGRAPHICLIB
option(GTSAM_ENABLE_GEOGRAPHICLIB "Enable Features that use GeographicLib" OFF)

if (GTSAM_ENABLE_GEOGRAPHICLIB)
  include(cmake/HandleGeographicLib.cmake)
endif()
# ##############################################################################

# Other Libraries:
include(cmake/HandleCCache.cmake)           # ccache
include(cmake/HandleCPack.cmake)            # CPack
include(cmake/HandleEigen.cmake)            # Eigen3
include(cmake/HandleMetis.cmake)            # metis
include(cmake/HandleCephes.cmake)           # cephes
include(cmake/HandleMKL.cmake)              # MKL
include(cmake/HandleOpenMP.cmake)           # OpenMP
include(cmake/HandlePerfTools.cmake)        # Google perftools
include(cmake/HandlePython.cmake)           # Python options and commands
include(cmake/HandleTBB.cmake)              # TBB
include(cmake/HandleUninstall.cmake)        # for "make uninstall"

include(cmake/HandleAllocators.cmake)       # Must be after tbb, pertools

include(cmake/HandleGlobalBuildFlags.cmake) # Build flags

###############################################################################
# Add components

# Build CppUnitLite
add_subdirectory(CppUnitLite)

# Build GTSAM library
add_subdirectory(gtsam)

# Build Tests
add_subdirectory(tests)

# Build examples
add_subdirectory(examples)

# Build timing
add_subdirectory(timing)

# Build gtsam_unstable
if (GTSAM_BUILD_UNSTABLE)
  add_subdirectory(gtsam_unstable)
endif()

# This is the new wrapper
if(GTSAM_BUILD_PYTHON OR GTSAM_INSTALL_MATLAB_TOOLBOX)
    # Need to set this for the wrap package so we don't use the default value.
    set(WRAP_PYTHON_VERSION ${GTSAM_PYTHON_VERSION}
        CACHE STRING "The Python version to use for wrapping")
    # Set the include directory for matlab.h
    set(GTWRAP_INCLUDE_NAME "wrap")

    # Copy matlab.h to the correct folder.
    configure_file(${PROJECT_SOURCE_DIR}/wrap/matlab.h
               ${PROJECT_BINARY_DIR}/wrap/matlab.h COPYONLY)

    add_subdirectory(wrap)
    list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/wrap/cmake")
endif()

# Python toolbox
if(GTSAM_BUILD_PYTHON)
    add_subdirectory(python)
endif()

# Matlab toolbox
if(GTSAM_INSTALL_MATLAB_TOOLBOX)
    add_subdirectory(matlab)
endif()

# Install config and export files
GtsamMakeConfigFile(GTSAM "${CMAKE_CURRENT_SOURCE_DIR}/gtsam_extra.cmake.in")
export(TARGETS ${GTSAM_EXPORTED_TARGETS} FILE GTSAM-exports.cmake)

if (GTSAM_BUILD_UNSTABLE)
    GtsamMakeConfigFile(GTSAM_UNSTABLE "${CMAKE_CURRENT_SOURCE_DIR}/gtsam_extra.cmake.in")
    export(TARGETS ${GTSAM_UNSTABLE_EXPORTED_TARGETS} FILE GTSAM_UNSTABLE-exports.cmake)
endif()

# Check for doxygen availability - optional dependency
find_package(Doxygen)

# Doxygen documentation - enabling options in subfolder
if (DOXYGEN_FOUND)
    add_subdirectory(doc)
endif()

# CMake Tools
add_subdirectory(cmake)

# Print configuration variables
include(cmake/HandlePrintConfiguration.cmake)

# Print warnings at the end
include(cmake/HandleFinalChecks.cmake)

# Include CPack *after* all flags
include(CPack)