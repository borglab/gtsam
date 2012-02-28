project(GTSAM CXX C)
cmake_minimum_required(VERSION 2.6)

# Set the version number for the library
set (GTSAM_VERSION_MAJOR 0)
set (GTSAM_VERSION_MINOR 9)
set (GTSAM_VERSION_PATCH 3)

# Set the default install path to home
set (CMAKE_INSTALL_PREFIX ${HOME} CACHE DOCSTRING "Install prefix for library")

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

# Add debugging flags
set(CMAKE_C_FLAGS_DEBUG            "${CMAKE_C_FLAGS_DEBUG} -fno-inline -Wall")
set(CMAKE_CXX_FLAGS_DEBUG          "${CMAKE_CXX_FLAGS_DEBUG} -fno-inline -Wall")
set(CMAKE_C_FLAGS_RELWITHDEBINFO   "-g -fno-inline -Wall -DNDEBUG -DEIGEN_NO_DEBUG")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-g -fno-inline -Wall -DNDEBUG -DEIGEN_NO_DEBUG")
set(CMAKE_C_FLAGS_RELEASE          "${CMAKE_C_FLAGS_RELEASE} -DNDEBUG -Wall -DEIGEN_NO_DEBUG")
set(CMAKE_CXX_FLAGS_RELEASE        "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG -Wall -DEIGEN_NO_DEBUG")

# Configurable Options
option(GTSAM_BUILD_TESTS    "Enable/Disable building of tests"          ON)
option(GTSAM_BUILD_TIMING   "Enable/Disable building of timing scripts" ON)
option(GTSAM_BUILD_EXAMPLES "Enable/Disable building of examples"       ON)
option(GTSAM_BUILD_WRAP     "Enable/Disable building of matlab wrap utility (necessary for matlab interface)" ON)
option(GTSAM_USE_QUATERNIONS "Enable/Disable using an internal Quaternion representation for rotations instead of rotation matrices" OFF)
option(GTSAM_BUILD_CONVENIENCE_LIBRARIES "Enable/Disable use of convenience libraries for faster development rebuilds, but slower install" ON)

# Add the Quaternion Build Flag if requested
if (GTSAM_USE_QUATERNIONS)
	set(CMAKE_C_FLAGS            "${CMAKE_C_FLAGS} -DGTSAM_DEFAULT_QUATERNIONS")
	set(CMAKE_CXX_FLAGS          "${CMAKE_CXX_FLAGS} -DGTSAM_DEFAULT_QUATERNIONS")
endif(GTSAM_USE_QUATERNIONS)

# Avoid building non-installed exes and unit tests when installing
# FIXME: breaks generation and install of matlab toolbox
#set(CMAKE_SKIP_INSTALL_ALL_DEPENDENCY TRUE)

# Pull in infrastructure
if (GTSAM_BUILD_TESTS)
    enable_testing()
    include(Dart)
    include(CTest)
endif()

# Use macros for creating tests/timing scripts
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include(GtsamTesting)

# Enable make check (http://www.cmake.org/Wiki/CMakeEmulateMakeCheck)
add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND})
add_custom_target(timing)

# Find boost
find_package(Boost 1.40 COMPONENTS serialization REQUIRED)

# General build settings
include_directories(
  gtsam/3rdparty/UFconfig 
  gtsam/3rdparty/CCOLAMD/Include
  ${CMAKE_SOURCE_DIR}
  CppUnitLite
  ${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIRS})

# Build CppUnitLite
add_subdirectory(CppUnitLite)

# Build GTSAM library
add_subdirectory(gtsam)

# Build Tests
add_subdirectory(tests)

# Build wrap
if (GTSAM_BUILD_WRAP)
    add_subdirectory(wrap)
endif(GTSAM_BUILD_WRAP)

# Build examples
if (GTSAM_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif(GTSAM_BUILD_EXAMPLES)

# Set up CPack
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "GTSAM")
set(CPACK_PACKAGE_VENDOR "Frank Dellaert, Georgia Institute of Technology")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_PACKAGE_VERSION_MAJOR ${GTSAM_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${GTSAM_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${GTSAM_VERSION_PATCH})
set(CPACK_PACKAGE_INSTALL_DIRECTORY "CMake ${CMake_VERSION_MAJOR}.${CMake_VERSION_MINOR}")
set(CPACK_INSTALLED_DIRECTORIES "doc" ".") # Include doc directory
set(CPACK_SOURCE_IGNORE_FILES "/build;/\\\\.;/makedoc.sh$")
set(CPACK_SOURCE_PACKAGE_FILE_NAME "gtsam-${GTSAM_VERSION_MAJOR}.${GTSAM_VERSION_MINOR}.${GTSAM_VERSION_PATCH}")
set(CPACK_SOURCE_GENERATOR "TGZ")
set(CPACK_GENERATOR "TGZ")
include(CPack)
