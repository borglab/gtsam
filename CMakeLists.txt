
project(GTSAM CXX C)
cmake_minimum_required(VERSION 2.6)

# Set the version number for the library
set (GTSAM_VERSION_MAJOR 2)
set (GTSAM_VERSION_MINOR 1)
set (GTSAM_VERSION_PATCH 0)

# Set the default install path to home
#set (CMAKE_INSTALL_PREFIX ${HOME} CACHE PATH "Install prefix for library")

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(GtsamMakeConfigFile)

# Record the root dir for gtsam - needed during external builds, e.g., ROS
set(GTSAM_SOURCE_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS "GTSAM_SOURCE_ROOT_DIR: [${GTSAM_SOURCE_ROOT_DIR}]")

# Load build type flags and default to Debug mode
include(GtsamBuildTypes)

# Use macros for creating tests/timing scripts
include(GtsamTesting)
include(GtsamPrinting)

# guard against in-source builds
if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
  message(FATAL_ERROR "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there. You may need to remove CMakeCache.txt. ")
endif()

# See whether gtsam_unstable is available (it will be present only if we're using an SVN checkout)
if(EXISTS "${PROJECT_SOURCE_DIR}/gtsam_unstable" AND IS_DIRECTORY "${PROJECT_SOURCE_DIR}/gtsam_unstable")
    set(GTSAM_UNSTABLE_AVAILABLE 1)
else()
    set(GTSAM_UNSTABLE_AVAILABLE 0)
endif()

# Configurable Options
option(GTSAM_BUILD_TESTS                 "Enable/Disable building of tests"          ON)
#option(GTSAM_BUILD_TIMING                "Enable/Disable building of timing scripts" ON) # These do not currently work
option(GTSAM_BUILD_EXAMPLES              "Enable/Disable building of examples"       ON)
if(GTSAM_UNSTABLE_AVAILABLE)
    option(GTSAM_BUILD_UNSTABLE              "Enable/Disable libgtsam_unstable"          ON)
endif()
if(NOT MSVC)
  option(GTSAM_BUILD_SHARED_LIBRARY        "Enable/Disable building of a shared version of gtsam" ON)
  option(GTSAM_BUILD_STATIC_LIBRARY        "Enable/Disable building of a static version of gtsam" ON)
else()
  set(GTSAM_BUILD_STATIC_LIBRARY ON)
endif()
option(GTSAM_USE_QUATERNIONS             "Enable/Disable using an internal Quaternion representation for rotations instead of rotation matrices" OFF)
if(NOT MSVC)
  option(GTSAM_BUILD_CONVENIENCE_LIBRARIES "Enable/Disable use of convenience libraries for faster development rebuilds, but slower install" ON)
endif()


# Options relating to MATLAB wrapper
# TODO: Check for matlab mex binary before handling building of binaries
option(GTSAM_INSTALL_MATLAB_TOOLBOX      "Enable/Disable installation of matlab toolbox"  OFF)
option(GTSAM_BUILD_WRAP                  "Enable/Disable building of matlab wrap utility (necessary for matlab interface)" ON)
option(GTSAM_INSTALL_WRAP                "Enable/Disable installation of wrap utility for wrapping other libraries"    ON)
set(GTSAM_TOOLBOX_INSTALL_PATH "" CACHE PATH "Matlab toolbox destination, blank defaults to CMAKE_INSTALL_PREFIX/borg/toolbox")
set(GTSAM_BUILD_MEX_BINARY_FLAGS "" CACHE STRING "Extra flags for running Matlab MEX compilation")
set(MEX_COMMAND "mex" CACHE FILEPATH "Command to use for executing mex (if on path, 'mex' will work)")

# Check / set dependent variables for MATLAB wrapper
set(GTSAM_WRAP_HEADER_PATH "${PROJECT_SOURCE_DIR}/wrap")
if(GTSAM_INSTALL_MATLAB_TOOLBOX AND NOT GTSAM_BUILD_WRAP)
	message(FATAL_ERROR "GTSAM_INSTALL_MATLAB_TOOLBOX is enabled, please also enable GTSAM_BUILD_WRAP")
endif()
if(GTSAM_INSTALL_WRAP AND NOT GTSAM_BUILD_WRAP)
	message(FATAL_ERROR "GTSAM_INSTALL_WRAP is enabled, please also enable GTSAM_BUILD_WRAP")
endif()
if(NOT GTSAM_TOOLBOX_INSTALL_PATH)
	set(GTSAM_TOOLBOX_INSTALL_PATH "${CMAKE_INSTALL_PREFIX}/borg/toolbox")
endif()


# Flags for choosing default packaging tools
set(CPACK_SOURCE_GENERATOR "TGZ" CACHE STRING "CPack Default Source Generator")
set(CPACK_GENERATOR        "TGZ" CACHE STRING "CPack Default Binary Generator")

# Sanity check building of libraries
if (NOT GTSAM_BUILD_SHARED_LIBRARY AND NOT GTSAM_BUILD_STATIC_LIBRARY)
    message(FATAL_ERROR "Both shared and static version of GTSAM library disabled - need to choose at least one!")
endif()

# Add the Quaternion Build Flag if requested
if (GTSAM_USE_QUATERNIONS)
  set(CMAKE_C_FLAGS            "${CMAKE_C_FLAGS} -DGTSAM_DEFAULT_QUATERNIONS")
  set(CMAKE_CXX_FLAGS          "${CMAKE_CXX_FLAGS} -DGTSAM_DEFAULT_QUATERNIONS")
endif(GTSAM_USE_QUATERNIONS)

# Flags to determine whether tests and examples are build during 'make install'
# Note that these remove the targets from the 'all'
option(GTSAM_DISABLE_TESTS_ON_INSTALL "Disables building tests during install" ON)
option(GTSAM_DISABLE_EXAMPLES_ON_INSTALL "Disables building examples during install" OFF)

# Pull in infrastructure
if (GTSAM_BUILD_TESTS)
    enable_testing()
    include(Dart)
    include(CTest)
endif()

# Find boost
if(CYGWIN OR MSVC OR WIN32)
  set(Boost_USE_STATIC_LIBS 1)
endif()
find_package(Boost 1.43 COMPONENTS serialization system filesystem thread date_time regex REQUIRED)
set(GTSAM_BOOST_LIBRARIES ${Boost_SERIALIZATION_LIBRARY} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})

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

# Build wrap
if (GTSAM_BUILD_WRAP)
    add_subdirectory(wrap)
endif(GTSAM_BUILD_WRAP)

# Build GTSAM library
add_subdirectory(gtsam)

# Build Tests
add_subdirectory(tests)

# Build examples
if (GTSAM_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif(GTSAM_BUILD_EXAMPLES)

# Matlab toolbox
if (GTSAM_INSTALL_MATLAB_TOOLBOX)
	add_subdirectory(matlab)
endif()

# Build gtsam_unstable
if (GTSAM_BUILD_UNSTABLE)
    add_subdirectory(gtsam_unstable)
endif(GTSAM_BUILD_UNSTABLE)

# Install config and export files
GtsamMakeConfigFile(GTSAM)
export(TARGETS ${GTSAM_EXPORTED_TARGETS} FILE GTSAM-exports.cmake)

# Check for doxygen availability - optional dependency
find_package(Doxygen)

# Doxygen documentation - enabling options in subfolder
if (DOXYGEN_FOUND)
    add_subdirectory(doc)
endif()

## Set up CPack
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "GTSAM")
set(CPACK_PACKAGE_VENDOR "Frank Dellaert, Georgia Institute of Technology")
set(CPACK_PACKAGE_CONTACT "Frank Dellaert, dellaert@cc.gatech.edu")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README")
set(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/LICENSE")
set(CPACK_PACKAGE_VERSION_MAJOR ${GTSAM_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${GTSAM_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${GTSAM_VERSION_PATCH})
set(CPACK_PACKAGE_INSTALL_DIRECTORY "CMake ${CMake_VERSION_MAJOR}.${CMake_VERSION_MINOR}")
#set(CPACK_INSTALLED_DIRECTORIES "doc;.") # Include doc directory
#set(CPACK_INSTALLED_DIRECTORIES ".") # FIXME: throws error
set(CPACK_SOURCE_IGNORE_FILES "/build*;/\\\\.;/makestats.sh$")
set(CPACK_SOURCE_IGNORE_FILES "${CPACK_SOURCE_IGNORE_FILES}" "/gtsam_unstable/")
set(CPACK_SOURCE_IGNORE_FILES "${CPACK_SOURCE_IGNORE_FILES}" "/package_scripts/")
set(CPACK_SOURCE_PACKAGE_FILE_NAME "gtsam-${GTSAM_VERSION_MAJOR}.${GTSAM_VERSION_MINOR}.${GTSAM_VERSION_PATCH}")
#set(CPACK_SOURCE_PACKAGE_FILE_NAME "gtsam-aspn${GTSAM_VERSION_PATCH}") # Used for creating ASPN tarballs

# Deb-package specific cpack
set(CPACK_DEBIAN_PACKAGE_NAME "libgtsam-dev")
set(CPACK_DEBIAN_PACKAGE_DEPENDS "libboost-dev (>= 1.43)") #Example: "libc6 (>= 2.3.1-6), libgcc1 (>= 1:3.4.2-12)")

# print configuration variables
message(STATUS "===============================================================")
message(STATUS "================  Configuration Options  ======================")
message(STATUS "Build flags                                               ")
#print_config_flag(${GTSAM_BUILD_TIMING}                "Build Timing scripts           ")
print_config_flag(${GTSAM_BUILD_EXAMPLES}              "Build Examples                 ")
print_config_flag(${GTSAM_BUILD_TESTS}                 "Build Tests                    ")
if (DOXYGEN_FOUND)
    print_config_flag(${GTSAM_BUILD_DOCS}                  "Build Docs                     ")
endif()
if(NOT MSVC)
	print_config_flag(${GTSAM_BUILD_SHARED_LIBRARY}        "Build shared GTSAM Library     ")
	print_config_flag(${GTSAM_BUILD_STATIC_LIBRARY}        "Build static GTSAM Library     ")
	print_config_flag(${GTSAM_BUILD_CONVENIENCE_LIBRARIES} "Build Convenience Libraries    ")
endif()
print_config_flag(${GTSAM_BUILD_TYPE_POSTFIXES}        "Put build-type in library name ")
if(GTSAM_UNSTABLE_AVAILABLE)
    print_config_flag(${GTSAM_BUILD_UNSTABLE}           "Build libgtsam_unstable       ")
endif()
print_config_flag(${GTSAM_DISABLE_TESTS_ON_INSTALL}     "Tests excluded from all/install target ")
print_config_flag(${GTSAM_DISABLE_EXAMPLES_ON_INSTALL}  "Examples excluded from all/install target ")
string(TOUPPER "${CMAKE_BUILD_TYPE}" cmake_build_type_toupper)
message(STATUS                                       "  Build type                 : ${CMAKE_BUILD_TYPE}")
message(STATUS                                       "  C compilation flags        : ${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_${cmake_build_type_toupper}}")
message(STATUS                                       "  C++ compilation flags      : ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${cmake_build_type_toupper}}")

message(STATUS "Packaging flags                                               ")
message(STATUS                                       "  CPack Source Generator     : ${CPACK_SOURCE_GENERATOR}")
message(STATUS                                       "  CPack Generator            : ${CPACK_GENERATOR}")

message(STATUS "GTSAM flags                                               ")
print_config_flag(${GTSAM_USE_QUATERNIONS}             "Quaternions as default Rot3")

message(STATUS "MATLAB toolbox flags                                      ")
print_config_flag(${GTSAM_INSTALL_MATLAB_TOOLBOX}      "Install matlab toolbox     ")
print_config_flag(${GTSAM_BUILD_WRAP}                  "Build Wrap                 ")
print_config_flag(${GTSAM_INSTALL_WRAP}                "Install wrap utility       ")
message(STATUS "===============================================================")

# Include CPack *after* all flags
include(CPack)
