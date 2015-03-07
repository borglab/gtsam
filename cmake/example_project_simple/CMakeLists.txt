# This file should be used as a template for creating new projects using the CMake tools
# This project has the following features
#    - GTSAM linking
#    - Unit tests via CppUnitLite
#    - Scripts

###################################################################################
# To create your own project, replace "myproject" with the actual name of your project
cmake_minimum_required(VERSION 2.6)
enable_testing()
project(myproject CXX C)               

# Add the cmake subfolder to the cmake module path - necessary to use macros
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

# Ensure that local folder is searched before library folders
include_directories(BEFORE "${PROJECT_SOURCE_DIR}")

# Load build type flags and default to Debug mode
include(GtsamBuildTypes)

###################################################################################
# Find GTSAM components
find_package(GTSAM REQUIRED) # Uses installed package
include_directories(${GTSAM_INCLUDE_DIR})

###################################################################################
# Build static library from common sources
add_library(${PROJECT_NAME} STATIC ${PROJECT_NAME}/MySourceFiles.cpp)
target_link_libraries(${PROJECT_NAME} gtsam-shared)

###################################################################################
# Build tests (CMake tracks the dependecy to link with GTSAM through our project's static library)
gtsam_add_subdir_tests(${PROJECT_NAME} "${PROJECT_NAME}" "${PROJECT_NAME}" "")

###################################################################################
# Build scripts (CMake tracks the dependecy to link with GTSAM through our project's static library)
gtsam_add_executables("${PROJECT_NAME}/myScripts.cpp" "${PROJECT_NAME}" "${PROJECT_NAME}" "")
