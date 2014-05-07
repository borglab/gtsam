# This file should be used as a template for creating new projects using the CMake tools
# This project has the following features
#    - GTSAM linking
#    - Unit tests via CppUnitLite
#    - Scripts
#    - Automatic MATLAB wrapper generation

###################################################################################
# To create your own project, replace "example" with the actual name of your project
cmake_minimum_required(VERSION 2.6)
project(example CXX C)

# Include GTSAM CMake tools
find_package(GTSAMCMakeTools)
include(GtsamBuildTypes) # Load build type flags and default to Debug mode
include(GtsamTesting)    # Easy functions for creating unit tests and scripts
include(GtsamMatlabWrap) # Automatic MATLAB wrapper generation

# Ensure that local folder is searched before library folders
include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

###################################################################################
# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})

###################################################################################
# Build static library from common sources
set(CONVENIENCE_LIB_NAME ${PROJECT_NAME})
add_library(${CONVENIENCE_LIB_NAME} STATIC example/PrintExamples.h example/PrintExamples.cpp)
target_link_libraries(${CONVENIENCE_LIB_NAME} gtsam)

###################################################################################
# Build tests (CMake tracks the dependecy to link with GTSAM through our project's static library)
gtsamAddTestsGlob("example" "tests/test*.cpp" "" "${CONVENIENCE_LIB_NAME}")

###################################################################################
# Build scripts (CMake tracks the dependecy to link with GTSAM through our project's static library)
gtsamAddExamplesGlob("*.cpp" "" "${CONVENIENCE_LIB_NAME}")

###################################################################################
# Build MATLAB wrapper (CMake tracks the dependecy to link with GTSAM through our project's static library)
wrap_and_install_library("example.h" "${CONVENIENCE_LIB_NAME}" "" "")
