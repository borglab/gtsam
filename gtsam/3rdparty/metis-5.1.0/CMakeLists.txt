cmake_minimum_required(VERSION 2.8)
project(METIS)

# Add flags for currect directory and below
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  add_definitions(-Wno-unused-variable)
  if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 5.0 OR CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 5.0)
    add_definitions(-Wno-sometimes-uninitialized)
  endif()
endif()

add_definitions(-Wno-unknown-pragmas)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.6 OR CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 4.6)
    add_definitions(-Wno-unused-but-set-variable)
  endif()
endif()

set(GKLIB_PATH ${PROJECT_SOURCE_DIR}/GKlib CACHE PATH "path to GKlib")
set(SHARED FALSE CACHE BOOL "build a shared library")

if(MSVC)
  set(METIS_INSTALL FALSE)
else()
  set(METIS_INSTALL FALSE)
endif()

# Configure libmetis library.
if(SHARED)
  set(METIS_LIBRARY_TYPE SHARED)
else()
  set(METIS_LIBRARY_TYPE STATIC)
endif(SHARED)

include(${GKLIB_PATH}/GKlibSystem.cmake)
# Add include directories.
include_directories(${GKLIB_PATH})
include_directories(include)
# Recursively look for CMakeLists.txt in subdirs.
add_subdirectory("include")
add_subdirectory("libmetis")

if(GTSAM_BUILD_METIS_EXECUTABLES)
  add_subdirectory("programs")
endif()
