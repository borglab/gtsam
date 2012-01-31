project(gtsam CXX C)
cmake_minimum_required(VERSION 2.6)

# Set the version number for the libarary
set (GTSAM_VERSION_MAJOR 0)
set (GTSAM_VERSION_MINOR 9)
set (GTSAM_VERSION_PATCH 3)

# guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. ")
endif()

# guard against bad build-type strings
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Debug")
endif()

string(TOLOWER "${CMAKE_BUILD_TYPE}" cmake_build_type_tolower)
if(    NOT cmake_build_type_tolower STREQUAL "debug"
   AND NOT cmake_build_type_tolower STREQUAL "release"
   AND NOT cmake_build_type_tolower STREQUAL "relwithdebinfo")
  message(FATAL_ERROR "Unknown build type \"${CMAKE_BUILD_TYPE}\". Allowed values are Debug, Release, RelWithDebInfo (case-insensitive).")
endif()

# Turn off function inlining when debugging
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fno-inline -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fno-inline -Wall")
# No optimization in relwithdebinfo
set(CMAKE_C_FLAGS_RELWITHDEBINFO "-g -fno-inline -Wall")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-g -fno-inline -Wall")
# Eigen no debug in release mode
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -DEIGEN_NO_DEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DEIGEN_NO_DEBUG")

# Configurable Options
# TODO

# Pull in tests
enable_testing()
include(Dart)
include(CTest)

# Enable make check (http://www.cmake.org/Wiki/CMakeEmulateMakeCheck)
add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND})
add_custom_target(timing)

# Find boost
find_package(Boost 1.40 REQUIRED)

# General build settings
include_directories(
  gtsam/3rdparty/UFconfig 
  gtsam/3rdparty/CCOLAMD/Include
  ${CMAKE_SOURCE_DIR}
  CppUnitLite
  ${BOOST_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

# Build GTSAM library
add_subdirectory(gtsam)

# Build CppUnitLite
add_subdirectory(CppUnitLite)

# Build Tests
#add_subdirectory(tests)

# Build wrap
#add_subdirectory(wrap)

# Build examples
#add_subdirectory(examples)
