cmake_minimum_required(VERSION 3.0)
project(METIS)

# Add flags for currect directory and below
if (${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
  add_definitions(-Wno-unused-variable)
  if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 5.0 OR CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 5.0)
    add_definitions(-Wno-sometimes-uninitialized)
  endif()
endif()

if(NOT (${CMAKE_C_COMPILER_ID} MATCHES "MSVC" OR ${CMAKE_CXX_COMPILER_ID} MATCHES "MSVC"))
  #add_definitions(-Wno-unknown-pragmas)
endif()

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 4.6 OR CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL 4.6)
    add_definitions(-Wno-unused-but-set-variable)
  endif()
endif()

set(GKLIB_PATH ${PROJECT_SOURCE_DIR}/GKlib CACHE PATH "path to GKlib")

if(MSVC)
  set(METIS_INSTALL FALSE)
else()
  set(METIS_INSTALL FALSE)
endif()

# Configure libmetis library.
if(BUILD_SHARED_LIBS)
  set(METIS_LIBRARY_TYPE SHARED)
else()
  set(METIS_LIBRARY_TYPE STATIC)
endif()

# Allow a static METIS while building GTSAM as dynamic
if(BUILD_STATIC_METIS)
  set(METIS_LIBRARY_TYPE STATIC)
endif()

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
# Export macros assumed in metis public headers to clients of the library.
# This was added to solve MSVC build errors.
if (TARGET metis AND GKlib_COPTIONS)
  # Remove (possibly) duplicated symbols:
  string(REPLACE -DLINUX "" GKlib_COPTIONS ${GKlib_COPTIONS})
  string(REPLACE -DWIN32 "" GKlib_COPTIONS ${GKlib_COPTIONS})
  string(REPLACE -DNDEBUG2 "" GKlib_COPTIONS ${GKlib_COPTIONS})
  string(REPLACE -DNDEBUG "" GKlib_COPTIONS ${GKlib_COPTIONS})
  string(REPLACE -pedantic "" GKlib_COPTIONS ${GKlib_COPTIONS})
  string(REPLACE -std=c99 "" GKlib_COPTIONS ${GKlib_COPTIONS})
  separate_arguments(GKlib_COPTIONS)
  # Declare those flags as to-be-imported in "client libraries", i.e. "gtsam"
  target_compile_options(metis INTERFACE ${GKlib_COPTIONS})
endif()
