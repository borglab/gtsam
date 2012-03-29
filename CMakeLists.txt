project(GTSAM CXX C)
cmake_minimum_required(VERSION 2.6)

# Set the version number for the library
set (GTSAM_VERSION_MAJOR 1)
set (GTSAM_VERSION_MINOR 9)
set (GTSAM_VERSION_PATCH 0)

# Set the default install path to home
set (CMAKE_INSTALL_PREFIX ${HOME} CACHE DOCSTRING "Install prefix for library")

# Use macros for creating tests/timing scripts
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
include(GtsamTesting)
include(GtsamPrinting)

# guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. ")
endif()

# Default to Debug mode
if(NOT FIRST_PASS_DONE)
  set(CMAKE_BUILD_TYPE "Debug" CACHE STRING
      "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
      FORCE)
endif()

# Check build types
if(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} VERSION_GREATER 2.8 OR ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} VERSION_EQUAL 2.8)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release Timing Profiling RelWithDebInfo MinSizeRel)
endif()
string(TOLOWER "${CMAKE_BUILD_TYPE}" cmake_build_type_tolower)
if(    NOT cmake_build_type_tolower STREQUAL "debug"
   AND NOT cmake_build_type_tolower STREQUAL "release"
   AND NOT cmake_build_type_tolower STREQUAL "timing"
   AND NOT cmake_build_type_tolower STREQUAL "profiling"
   AND NOT cmake_build_type_tolower STREQUAL "relwithdebinfo")
  message(FATAL_ERROR "Unknown build type \"${CMAKE_BUILD_TYPE}\". Allowed values are Debug, Release, Timing, Profiling, RelWithDebInfo (case-insensitive).")
endif()

# Add debugging flags but only on the first pass
if(NOT FIRST_PASS_DONE)
  set(CMAKE_C_FLAGS_DEBUG            "${CMAKE_C_FLAGS_DEBUG} -fno-inline -Wall" CACHE STRING "Flags used by the compiler during debug builds." FORCE)
  set(CMAKE_CXX_FLAGS_DEBUG          "${CMAKE_CXX_FLAGS_DEBUG} -fno-inline -Wall" CACHE STRING "Flags used by the compiler during debug builds." FORCE)
  set(CMAKE_C_FLAGS_RELWITHDEBINFO   "-g -fno-inline -Wall -DNDEBUG" CACHE STRING "Flags used by the compiler during relwithdebinfo builds." FORCE)
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-g -fno-inline -Wall -DNDEBUG" CACHE STRING "Flags used by the compiler during relwithdebinfo builds." FORCE)
  set(CMAKE_C_FLAGS_RELEASE          "${CMAKE_C_FLAGS_RELEASE} -Wall" CACHE STRING "Flags used by the compiler during release builds." FORCE)
  set(CMAKE_CXX_FLAGS_RELEASE        "${CMAKE_CXX_FLAGS_RELEASE} -Wall" CACHE STRING "Flags used by the compiler during release builds." FORCE)
  set(CMAKE_C_FLAGS_TIMING           "${CMAKE_C_FLAGS_RELEASE} -DENABLE_TIMING" CACHE STRING "Flags used by the compiler during timing builds." FORCE)
  set(CMAKE_CXX_FLAGS_TIMING         "${CMAKE_CXX_FLAGS_RELEASE} -DENABLE_TIMING" CACHE STRING "Flags used by the compiler during timing builds." FORCE)
  mark_as_advanced(CMAKE_C_FLAGS_TIMING CMAKE_CXX_FLAGS_TIMING)
  set(CMAKE_C_FLAGS_PROFILING        "-g -O2 -Wall -DNDEBUG" CACHE STRING "Flags used by the compiler during profiling builds." FORCE)
  set(CMAKE_CXX_FLAGS_PROFILING      "-g -O2 -Wall -DNDEBUG" CACHE STRING "Flags used by the compiler during profiling builds." FORCE)
  mark_as_advanced(CMAKE_C_FLAGS_PROFILING CMAKE_CXX_FLAGS_PROFILING)
endif()

# Configurable Options
option(GTSAM_BUILD_TESTS    "Enable/Disable building of tests"          ON)
option(GTSAM_BUILD_TIMING   "Enable/Disable building of timing scripts" ON)
option(GTSAM_BUILD_EXAMPLES "Enable/Disable building of examples"       ON)
option(GTSAM_BUILD_WRAP     "Enable/Disable building of matlab wrap utility (necessary for matlab interface)" ON)
option(GTSAM_USE_QUATERNIONS "Enable/Disable using an internal Quaternion representation for rotations instead of rotation matrices" OFF)
option(GTSAM_BUILD_CONVENIENCE_LIBRARIES "Enable/Disable use of convenience libraries for faster development rebuilds, but slower install" ON)
option(GTSAM_INSTALL_MATLAB_TOOLBOX  "Enable/Disable installation of matlab toolbox"  ON)
option(GTSAM_INSTALL_MATLAB_EXAMPLES "Enable/Disable installation of matlab examples" ON)
option(GTSAM_INSTALL_MATLAB_TESTS    "Enable/Disable installation of matlab tests"    ON)
option(GTSAM_INSTALL_WRAP "Enable/Disable installation of wrap utility" ON)

# Add the Quaternion Build Flag if requested
if (GTSAM_USE_QUATERNIONS)
	set(CMAKE_C_FLAGS            "${CMAKE_C_FLAGS} -DGTSAM_DEFAULT_QUATERNIONS")
	set(CMAKE_CXX_FLAGS          "${CMAKE_CXX_FLAGS} -DGTSAM_DEFAULT_QUATERNIONS")
endif(GTSAM_USE_QUATERNIONS)

# Avoid building non-installed exes and unit tests when installing
# FIXME: breaks generation and install of matlab toolbox
# FIXME: can't add install dependencies, so libraries never get built
#set(CMAKE_SKIP_INSTALL_ALL_DEPENDENCY TRUE)

# Pull in infrastructure
if (GTSAM_BUILD_TESTS)
    enable_testing()
    include(Dart)
    include(CTest)
endif()

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

# Mark that first pass is done
set(FIRST_PASS_DONE true CACHE BOOL "Internally used to mark whether cmake has been run multiple times")
mark_as_advanced(FIRST_PASS_DONE) 

# print configuration variables
message(STATUS "===============================================================")
message(STATUS "================  Configuration Options  ======================")
message(STATUS "Build flags                                               ")
print_config_flag(${GTSAM_BUILD_TIMING}                "Build Timing scripts       ")
print_config_flag(${GTSAM_BUILD_EXAMPLES}              "Build Examples             ")
print_config_flag(${GTSAM_BUILD_TESTS}                 "Build Tests                ")
print_config_flag(${GTSAM_BUILD_WRAP}                  "Build Wrap                 ")
print_config_flag(${GTSAM_BUILD_CONVENIENCE_LIBRARIES} "Build Convenience Libraries")
string(TOUPPER "${CMAKE_BUILD_TYPE}" cmake_build_type_toupper)
message(STATUS                                       "  Build type                 : ${CMAKE_BUILD_TYPE}")
message(STATUS                                       "  C compilation flags        : ${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_${cmake_build_type_toupper}}")
message(STATUS                                       "  C++ compilation flags      : ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${cmake_build_type_toupper}}")

message(STATUS "GTSAM flags                                               ")
print_config_flag(${GTSAM_USE_QUATERNIONS}             "Quaternions as default Rot3")

message(STATUS "MATLAB toolbox flags                                      ")
print_config_flag(${GTSAM_INSTALL_MATLAB_TOOLBOX}      "Install matlab toolbox     ")
print_config_flag(${GTSAM_INSTALL_MATLAB_EXAMPLES}     "Install matlab examples    ")
print_config_flag(${GTSAM_INSTALL_MATLAB_TESTS}        "Install matlab tests       ")
print_config_flag(${GTSAM_INSTALL_WRAP}                "Install wrap utility       ")
message(STATUS "===============================================================")

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
#set(CPACK_SOURCE_PACKAGE_FILE_NAME "gtsam-aspn${GTSAM_VERSION_PATCH}") # Used for creating ASPN tarballs
set(CPACK_SOURCE_GENERATOR "TGZ")
set(CPACK_GENERATOR "TGZ")
include(CPack)
