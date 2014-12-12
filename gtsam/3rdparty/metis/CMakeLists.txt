cmake_minimum_required(VERSION 2.8)
project(METIS)

# Add flags for currect directory and below
if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  add_definitions(-Wno-unused-variable)
#  add_definitions(-Wno-sometimes-uninitialized)
endif()

add_definitions(-Wno-unknown-pragmas)
#add_definitions(-Wunused-but-set-variable)

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

set(GTSAM_EXPORTED_TARGETS "${GTSAM_EXPORTED_TARGETS}" PARENT_SCOPE)

