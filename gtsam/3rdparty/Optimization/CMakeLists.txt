# A simple CMakeLists file that enables the Optimization library to be conveniently imported by other CMake projects using
cmake_minimum_required(VERSION 3.5)

# PROJECT CONFIGURATION
project(Optimization LANGUAGES CXX VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 17) # We require C++ 17 or later
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set build type to 'RelWithDebInfo' if one was not specified by the user
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS RelWithDebInfo Release Debug MinSizeRel)
  message(STATUS "Setting build type to ${CMAKE_BUILD_TYPE}, as none was specified\n")
else()
  message(STATUS "Building in ${CMAKE_BUILD_TYPE} mode\n")
endif()

# BUILD CONFIGURATIONS
# Build the examples?
set(BUILD_OPTIMIZATION_EXAMPLES OFF CACHE BOOL "Build example executables? [disabled by default]")
# Build the unit tests?
set(BUILD_OPTIMIZATION_TESTS OFF CACHE BOOL "Build unit tests? [disabled by default]")

# SET UP EXPORT OF OPTIMIZATION LIBRARY

# We declare this library as an INTERFACE library; this means that it does not directly produce built output, though it may have properties set on it and it may be installed, exported and imported.
add_library(${PROJECT_NAME} INTERFACE)

# Set the include directory for this project.  We use include/ as the top-level include directory so that #include directives in client source files are of the form "${PROJECT_NAME}/blah/blah.h"
set(OPTIMIZATION_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
message(STATUS "Found ${PROJECT_NAME} include directory: ${OPTIMIZATION_INCLUDE_DIR}")
target_include_directories(${PROJECT_NAME} INTERFACE ${OPTIMIZATION_INCLUDE_DIR})

# Top-level directory of the Optimization library
set(OPTIMIZATION_TOPLEVEL_DIR ${OPTIMIZATION_INCLUDE_DIR}/Optimization)

# The list of header files provided by this project
set(OPTIMIZATION_HDRS
    ${OPTIMIZATION_TOPLEVEL_DIR}/Base/Concepts.h
    ${OPTIMIZATION_TOPLEVEL_DIR}/Util/Stopwatch.h
    ${OPTIMIZATION_TOPLEVEL_DIR}/Convex/Concepts.h
    ${OPTIMIZATION_TOPLEVEL_DIR}/Convex/ProximalGradient.h
    ${OPTIMIZATION_TOPLEVEL_DIR}/Convex/ADMM.h
    ${OPTIMIZATION_TOPLEVEL_DIR}/LinearAlgebra/Concepts.h
    ${OPTIMIZATION_TOPLEVEL_DIR}/LinearAlgebra/IterativeSolvers.h
    ${OPTIMIZATION_TOPLEVEL_DIR}/LinearAlgebra/LOBPCG.h
    ${OPTIMIZATION_TOPLEVEL_DIR}/Riemannian/Concepts.h
    ${OPTIMIZATION_TOPLEVEL_DIR}/Riemannian/GradientDescent.h
    ${OPTIMIZATION_TOPLEVEL_DIR}/Riemannian/TNT.h
    ${OPTIMIZATION_TOPLEVEL_DIR}/Riemannian/TNLS.h
    )
message(STATUS "Found ${PROJECT_NAME} header files:\n ${OPTIMIZATION_HDRS}")

# List the complete set of header files as source files.
target_sources(${PROJECT_NAME} INTERFACE ${OPTIMIZATION_HDRS})

# Export Optimization library

# Add add entry for this project into CMake's package registry, so that this project can be found by other CMake projects
export(PACKAGE ${PROJECT_NAME})
# Create a configuration file for this project, so that it can be imported by other CMake projects
export(TARGETS ${PROJECT_NAME} FILE ${PROJECT_NAME}Config.cmake)


# SET UP [OPTIONAL] CODE EXAMPLES AND TESTS

# Define some additional (optional) named targets, not built by default.  Typically, these will be e.g. unit tests, example executables.

if(${BUILD_OPTIMIZATION_EXAMPLES} OR ${BUILD_OPTIMIZATION_TESTS})
    # Directory for built libraries
    set(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/lib CACHE PATH "The directory in which to place libraries built by this project")

    # Directory for built executables
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/bin CACHE PATH "The directory in which to place executables built by this project")
endif()

# This is the name of the target in the generated Makefile that builds the example executables; i.e., to build these, one must type "make $EXAMPLES_TARGET_NAME" using the generated Makefile in order to build the example executables (we do this because we don't want these executables to be automatically built by other CMake projects that use this one as a dependency)
if(${BUILD_OPTIMIZATION_EXAMPLES})
message(STATUS "Adding examples to build")
add_subdirectory(examples)
endif()

# This is the name of the target in the generated Makefile that builds the tests; i.e., to build these, one must type "make $TESTS_TARGET_NAME" using the generated Makefile in order to build the example executables (we do this because we don't want these executables to be automatically built by other CMake projects that use this one as a dependency)
if(${BUILD_OPTIMIZATION_TESTS})
message(STATUS "Adding unit tests to build")
add_subdirectory(tests)
endif()
