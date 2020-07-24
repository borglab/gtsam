project(GTSAM CXX C)
cmake_minimum_required(VERSION 3.0)

# new feature to Cmake Version > 2.8.12
# Mac ONLY. Define Relative Path on Mac OS
if(NOT DEFINED CMAKE_MACOSX_RPATH)
  set(CMAKE_MACOSX_RPATH 0)
endif()

# Set the version number for the library
set (GTSAM_VERSION_MAJOR 4)
set (GTSAM_VERSION_MINOR 0)
set (GTSAM_VERSION_PATCH 3)
math (EXPR GTSAM_VERSION_NUMERIC "10000 * ${GTSAM_VERSION_MAJOR} + 100 * ${GTSAM_VERSION_MINOR} + ${GTSAM_VERSION_PATCH}")
set (GTSAM_VERSION_STRING "${GTSAM_VERSION_MAJOR}.${GTSAM_VERSION_MINOR}.${GTSAM_VERSION_PATCH}")

###############################################################################
# Gather information, perform checks, set defaults

# Set the default install path to home
#set (CMAKE_INSTALL_PREFIX ${HOME} CACHE PATH "Install prefix for library")

set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH}" "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(GtsamMakeConfigFile)
include(GNUInstallDirs)

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

# See whether gtsam_unstable is available (it will be present only if we're using a git checkout)
if(EXISTS "${PROJECT_SOURCE_DIR}/gtsam_unstable" AND IS_DIRECTORY "${PROJECT_SOURCE_DIR}/gtsam_unstable")
    set(GTSAM_UNSTABLE_AVAILABLE 1)
else()
    set(GTSAM_UNSTABLE_AVAILABLE 0)
endif()

# ----------------------------------------------------------------------------
#   Uninstall target, for "make uninstall"
# ----------------------------------------------------------------------------
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)

add_custom_target(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")


###############################################################################
# Set up options

# Configurable Options
if(GTSAM_UNSTABLE_AVAILABLE)
    option(GTSAM_BUILD_UNSTABLE              "Enable/Disable libgtsam_unstable"          ON)
endif()
option(BUILD_SHARED_LIBS                 "Build shared gtsam library, instead of static" ON)
option(GTSAM_USE_QUATERNIONS             "Enable/Disable using an internal Quaternion representation for rotations instead of rotation matrices. If enable, Rot3::EXPMAP is enforced by default." OFF)
option(GTSAM_POSE3_EXPMAP 			 	 "Enable/Disable using Pose3::EXPMAP as the default mode. If disabled, Pose3::FIRST_ORDER will be used." ON)
option(GTSAM_ROT3_EXPMAP 			 	 "Ignore if GTSAM_USE_QUATERNIONS is OFF (Rot3::EXPMAP by default). Otherwise, enable Rot3::EXPMAP, or if disabled, use Rot3::CAYLEY." ON)
option(GTSAM_ENABLE_CONSISTENCY_CHECKS   "Enable/Disable expensive consistency checks"       OFF)
option(GTSAM_WITH_TBB                    "Use Intel Threaded Building Blocks (TBB) if available" ON)
option(GTSAM_WITH_EIGEN_MKL              "Eigen will use Intel MKL if available" OFF)
option(GTSAM_WITH_EIGEN_MKL_OPENMP       "Eigen, when using Intel MKL, will also use OpenMP for multithreading if available" OFF)
option(GTSAM_THROW_CHEIRALITY_EXCEPTION  "Throw exception when a triangulated point is behind a camera" ON)
option(GTSAM_ALLOW_DEPRECATED_SINCE_V4   "Allow use of methods/functions deprecated in GTSAM 4" ON)
option(GTSAM_TYPEDEF_POINTS_TO_VECTORS   "Typedef Point2 and Point3 to Eigen::Vector equivalents" OFF)
option(GTSAM_SUPPORT_NESTED_DISSECTION   "Support Metis-based nested dissection" ON)
option(GTSAM_TANGENT_PREINTEGRATION      "Use new ImuFactor with integration on tangent space" ON)
if(NOT MSVC AND NOT XCODE_VERSION)
    option(GTSAM_BUILD_WITH_CCACHE           "Use ccache compiler cache" ON)
endif()

if(NOT MSVC AND NOT XCODE_VERSION)
  # Set the build type to upper case for downstream use
  string(TOUPPER "${CMAKE_BUILD_TYPE}" CMAKE_BUILD_TYPE_UPPER)

  # Set the GTSAM_BUILD_TAG variable.
  # If build type is Release, set to blank (""), else set to the build type.
  if(${CMAKE_BUILD_TYPE_UPPER} STREQUAL "RELEASE")
   set(GTSAM_BUILD_TAG "") # Don't create release mode tag on installed directory
  else()
   set(GTSAM_BUILD_TAG "${CMAKE_BUILD_TYPE}")
  endif()
endif()

# Options relating to MATLAB wrapper
# TODO: Check for matlab mex binary before handling building of binaries
option(GTSAM_INSTALL_MATLAB_TOOLBOX      "Enable/Disable installation of matlab toolbox"  OFF)
option(GTSAM_INSTALL_CYTHON_TOOLBOX      "Enable/Disable installation of Cython toolbox"  OFF)
option(GTSAM_BUILD_WRAP                  "Enable/Disable building of matlab/cython wrap utility (necessary for matlab/cython interface)" ON)
set(GTSAM_PYTHON_VERSION "Default" CACHE STRING "The version of python to build the cython wrapper for (or Default)")

# Check / set dependent variables for MATLAB wrapper
if((GTSAM_INSTALL_MATLAB_TOOLBOX OR GTSAM_INSTALL_CYTHON_TOOLBOX) AND NOT GTSAM_BUILD_WRAP)
	message(FATAL_ERROR "GTSAM_INSTALL_MATLAB_TOOLBOX or GTSAM_INSTALL_CYTHON_TOOLBOX is enabled, please also enable GTSAM_BUILD_WRAP")
endif()
if((GTSAM_INSTALL_MATLAB_TOOLBOX OR GTSAM_INSTALL_CYTHON_TOOLBOX) AND GTSAM_BUILD_TYPE_POSTFIXES)
		set(CURRENT_POSTFIX ${CMAKE_${CMAKE_BUILD_TYPE_UPPER}_POSTFIX})
endif()
if(GTSAM_INSTALL_WRAP AND NOT GTSAM_BUILD_WRAP)
	message(FATAL_ERROR "GTSAM_INSTALL_WRAP is enabled, please also enable GTSAM_BUILD_WRAP")
endif()

if(GTSAM_INSTALL_MATLAB_TOOLBOX AND NOT BUILD_SHARED_LIBS)
	message(FATAL_ERROR "GTSAM_INSTALL_MATLAB_TOOLBOX and BUILD_SHARED_LIBS=OFF. The MATLAB wrapper cannot be compiled with a static GTSAM library because mex modules are themselves shared libraries.  If you want a self-contained mex module, enable GTSAM_MEX_BUILD_STATIC_MODULE instead of BUILD_SHARED_LIBS=OFF.")
endif()

if(GTSAM_INSTALL_MATLAB_TOOLBOX AND GTSAM_TYPEDEF_POINTS_TO_VECTORS)
    message(FATAL_ERROR "GTSAM_INSTALL_MATLAB_TOOLBOX and GTSAM_TYPEDEF_POINTS_TO_VECTORS are both enabled. For now, the MATLAB toolbox cannot deal with this yet.  Please turn one of the two options off.")
endif()

if(GTSAM_INSTALL_CYTHON_TOOLBOX AND GTSAM_TYPEDEF_POINTS_TO_VECTORS)
    message(FATAL_ERROR "GTSAM_INSTALL_CYTHON_TOOLBOX and GTSAM_TYPEDEF_POINTS_TO_VECTORS are both enabled. For now, the CYTHON toolbox cannot deal with this yet.  Please turn one of the two options off.")
endif()

# Flags for choosing default packaging tools
set(CPACK_SOURCE_GENERATOR "TGZ" CACHE STRING "CPack Default Source Generator")
set(CPACK_GENERATOR        "TGZ" CACHE STRING "CPack Default Binary Generator")

###############################################################################
# Find boost

# To change the path for boost, you will need to set:
# BOOST_ROOT: path to install prefix for boost
# Boost_NO_SYSTEM_PATHS: set to true to keep the find script from ignoring BOOST_ROOT

if(MSVC)
	# By default, boost only builds static libraries on windows
	set(Boost_USE_STATIC_LIBS ON)  # only find static libs
	# If we ever reset above on windows and, ...
	# If we use Boost shared libs, disable auto linking.
	# Some libraries, at least Boost Program Options, rely on this to export DLL symbols.
	if(NOT Boost_USE_STATIC_LIBS)
		list_append_cache(GTSAM_COMPILE_DEFINITIONS_PUBLIC BOOST_ALL_NO_LIB BOOST_ALL_DYN_LINK)
	endif()
	# Virtual memory range for PCH exceeded on VS2015
	if(MSVC_VERSION LESS 1910) # older than VS2017
	  list_append_cache(GTSAM_COMPILE_OPTIONS_PRIVATE -Zm295)
	endif()
endif()

# If building DLLs in MSVC, we need to avoid EIGEN_STATIC_ASSERT()
# or explicit instantiation will generate build errors.
# See: https://bitbucket.org/gtborg/gtsam/issues/417/fail-to-build-on-msvc-2017
#
if(MSVC AND BUILD_SHARED_LIBS)
	list_append_cache(GTSAM_COMPILE_DEFINITIONS_PUBLIC EIGEN_NO_STATIC_ASSERT)
endif()

# Store these in variables so they are automatically replicated in GTSAMConfig.cmake and such.
set(BOOST_FIND_MINIMUM_VERSION 1.43)
set(BOOST_FIND_MINIMUM_COMPONENTS serialization system filesystem thread program_options date_time timer chrono regex)

find_package(Boost ${BOOST_FIND_MINIMUM_VERSION} COMPONENTS ${BOOST_FIND_MINIMUM_COMPONENTS})

# Required components
if(NOT Boost_SERIALIZATION_LIBRARY OR NOT Boost_SYSTEM_LIBRARY OR NOT Boost_FILESYSTEM_LIBRARY OR
    NOT Boost_THREAD_LIBRARY OR NOT Boost_DATE_TIME_LIBRARY)
  message(FATAL_ERROR "Missing required Boost components >= v1.43, please install/upgrade Boost or configure your search paths.")
endif()

option(GTSAM_DISABLE_NEW_TIMERS "Disables using Boost.chrono for timing" OFF)
# Allow for not using the timer libraries on boost < 1.48 (GTSAM timing code falls back to old timer library)
set(GTSAM_BOOST_LIBRARIES
  Boost::serialization
  Boost::system
  Boost::filesystem
  Boost::thread
  Boost::date_time
  Boost::regex
)
if (GTSAM_DISABLE_NEW_TIMERS)
    message("WARNING:  GTSAM timing instrumentation manually disabled")
    list_append_cache(GTSAM_COMPILE_DEFINITIONS_PUBLIC DGTSAM_DISABLE_NEW_TIMERS)
else()
    if(Boost_TIMER_LIBRARY)
      list(APPEND GTSAM_BOOST_LIBRARIES Boost::timer Boost::chrono)
    else()
      list(APPEND GTSAM_BOOST_LIBRARIES rt) # When using the header-only boost timer library, need -lrt
      message("WARNING:  GTSAM timing instrumentation will use the older, less accurate, Boost timer library because boost older than 1.48 was found.")
    endif()
endif()

###############################################################################
# Find TBB
find_package(TBB 4.4 COMPONENTS tbb tbbmalloc)

# Set up variables if we're using TBB
if(TBB_FOUND AND GTSAM_WITH_TBB)
    set(GTSAM_USE_TBB 1)  # This will go into config.h
    if ((${TBB_VERSION_MAJOR} GREATER 2020) OR (${TBB_VERSION_MAJOR} EQUAL 2020))
        set(TBB_GREATER_EQUAL_2020 1)
    else()
        set(TBB_GREATER_EQUAL_2020 0)
    endif()
    # all definitions and link requisites will go via imported targets:
    # tbb & tbbmalloc
    list(APPEND GTSAM_ADDITIONAL_LIBRARIES tbb tbbmalloc)
else()
    set(GTSAM_USE_TBB 0)  # This will go into config.h
endif()

###############################################################################
# Prohibit Timing build mode in combination with TBB
if(GTSAM_USE_TBB AND (CMAKE_BUILD_TYPE  STREQUAL "Timing"))
      message(FATAL_ERROR "Timing build mode cannot be used together with TBB. Use a sampling profiler such as Instruments or Intel VTune Amplifier instead.")
endif()


###############################################################################
# Find Google perftools
find_package(GooglePerfTools)

###############################################################################
# Support ccache, if installed
if(NOT MSVC AND NOT XCODE_VERSION)
	find_program(CCACHE_FOUND ccache)
	if(CCACHE_FOUND)
		if(GTSAM_BUILD_WITH_CCACHE)
			set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
			set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
		else()
			set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "")
			set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "")
		endif()
	endif(CCACHE_FOUND)
endif()

###############################################################################
# Find MKL
find_package(MKL)

if(MKL_FOUND AND GTSAM_WITH_EIGEN_MKL)
    set(GTSAM_USE_EIGEN_MKL 1) # This will go into config.h
    set(EIGEN_USE_MKL_ALL 1) # This will go into config.h - it makes Eigen use MKL
    list(APPEND GTSAM_ADDITIONAL_LIBRARIES ${MKL_LIBRARIES})

    # --no-as-needed is required with gcc according to the MKL link advisor
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--no-as-needed")
    endif()
else()
    set(GTSAM_USE_EIGEN_MKL 0)
    set(EIGEN_USE_MKL_ALL 0)
endif()

###############################################################################
# Find OpenMP (if we're also using MKL)
find_package(OpenMP)  # do this here to generate correct message if disabled

if(GTSAM_WITH_EIGEN_MKL AND GTSAM_WITH_EIGEN_MKL_OPENMP AND GTSAM_USE_EIGEN_MKL)
    if(OPENMP_FOUND AND GTSAM_USE_EIGEN_MKL AND GTSAM_WITH_EIGEN_MKL_OPENMP)
        set(GTSAM_USE_EIGEN_MKL_OPENMP 1) # This will go into config.h
        list_append_cache(GTSAM_COMPILE_OPTIONS_PUBLIC ${OpenMP_CXX_FLAGS})
    endif()
endif()


###############################################################################
# Option for using system Eigen or GTSAM-bundled Eigen
### These patches only affect usage of MKL. If you want to enable MKL, you *must*
### use our patched version of Eigen
### See:  http://eigen.tuxfamily.org/bz/show_bug.cgi?id=704 (Householder QR MKL selection)
###       http://eigen.tuxfamily.org/bz/show_bug.cgi?id=705 (Fix MKL LLT return code)
option(GTSAM_USE_SYSTEM_EIGEN "Find and use system-installed Eigen. If 'off', use the one bundled with GTSAM" OFF)
option(GTSAM_WITH_EIGEN_UNSUPPORTED "Install Eigen's unsupported modules" OFF)

# Switch for using system Eigen or GTSAM-bundled Eigen
if(GTSAM_USE_SYSTEM_EIGEN)
	find_package(Eigen3 REQUIRED)

	# Use generic Eigen include paths e.g. <Eigen/Core>
	set(GTSAM_EIGEN_INCLUDE_FOR_INSTALL "${EIGEN3_INCLUDE_DIR}")

	# check if MKL is also enabled - can have one or the other, but not both!
	# Note: Eigen >= v3.2.5 includes our patches
	if(EIGEN_USE_MKL_ALL AND (EIGEN3_VERSION VERSION_LESS 3.2.5))
	  message(FATAL_ERROR "MKL requires at least Eigen 3.2.5, and your system appears to have an older version. Disable GTSAM_USE_SYSTEM_EIGEN to use GTSAM's copy of Eigen, or disable GTSAM_WITH_EIGEN_MKL")
	endif()

	# Check for Eigen version which doesn't work with MKL
	# See http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1527 for details.
	if(EIGEN_USE_MKL_ALL AND (EIGEN3_VERSION VERSION_EQUAL 3.3.4))
		message(FATAL_ERROR "MKL does not work with Eigen 3.3.4 because of a bug in Eigen. See http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1527. Disable GTSAM_USE_SYSTEM_EIGEN to use GTSAM's copy of Eigen, disable GTSAM_WITH_EIGEN_MKL, or upgrade/patch your installation of Eigen.")
	endif()

	# The actual include directory (for BUILD cmake target interface):
	set(GTSAM_EIGEN_INCLUDE_FOR_BUILD "${EIGEN3_INCLUDE_DIR}")
else()
	# Use bundled Eigen include path.
	# Clear any variables set by FindEigen3
	if(EIGEN3_INCLUDE_DIR)
		set(EIGEN3_INCLUDE_DIR NOTFOUND CACHE STRING "" FORCE)
	endif()

	# set full path to be used by external projects
	# this will be added to GTSAM_INCLUDE_DIR by gtsam_extra.cmake.in
	set(GTSAM_EIGEN_INCLUDE_FOR_INSTALL "include/gtsam/3rdparty/Eigen/")

	# The actual include directory (for BUILD cmake target interface):
	set(GTSAM_EIGEN_INCLUDE_FOR_BUILD "${CMAKE_SOURCE_DIR}/gtsam/3rdparty/Eigen/")
endif()

# Detect Eigen version:
set(EIGEN_VER_H "${GTSAM_EIGEN_INCLUDE_FOR_BUILD}/Eigen/src/Core/util/Macros.h")
if (EXISTS ${EIGEN_VER_H})
	file(READ "${EIGEN_VER_H}" STR_EIGEN_VERSION)

	# Extract the Eigen version from the Macros.h file, lines "#define EIGEN_WORLD_VERSION  XX", etc...

	string(REGEX MATCH "EIGEN_WORLD_VERSION[ ]+[0-9]+" GTSAM_EIGEN_VERSION_WORLD "${STR_EIGEN_VERSION}")
	string(REGEX MATCH "[0-9]+" GTSAM_EIGEN_VERSION_WORLD "${GTSAM_EIGEN_VERSION_WORLD}")

	string(REGEX MATCH "EIGEN_MAJOR_VERSION[ ]+[0-9]+" GTSAM_EIGEN_VERSION_MAJOR "${STR_EIGEN_VERSION}")
	string(REGEX MATCH "[0-9]+" GTSAM_EIGEN_VERSION_MAJOR "${GTSAM_EIGEN_VERSION_MAJOR}")

	string(REGEX MATCH "EIGEN_MINOR_VERSION[ ]+[0-9]+" GTSAM_EIGEN_VERSION_MINOR "${STR_EIGEN_VERSION}")
	string(REGEX MATCH "[0-9]+" GTSAM_EIGEN_VERSION_MINOR "${GTSAM_EIGEN_VERSION_MINOR}")

	set(GTSAM_EIGEN_VERSION "${GTSAM_EIGEN_VERSION_WORLD}.${GTSAM_EIGEN_VERSION_MAJOR}.${GTSAM_EIGEN_VERSION_MINOR}")

	message(STATUS "Found Eigen version: ${GTSAM_EIGEN_VERSION}")
else()
	message(WARNING "Cannot determine Eigen version, missing file: `${EIGEN_VER_H}`")
endif ()

if (MSVC)
	if (BUILD_SHARED_LIBS)
		# mute eigen static assert to avoid errors in shared lib
		list_append_cache(GTSAM_COMPILE_DEFINITIONS_PUBLIC EIGEN_NO_STATIC_ASSERT)
	endif()
	list_append_cache(GTSAM_COMPILE_OPTIONS_PRIVATE "/wd4244") # Disable loss of precision which is thrown all over our Eigen
endif()

if (APPLE AND BUILD_SHARED_LIBS)
	# Set the default install directory on macOS
	set(CMAKE_INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}/lib")
endif()

###############################################################################
# Global compile options

# Build list of possible allocators
set(possible_allocators "")
if(GTSAM_USE_TBB)
	list(APPEND possible_allocators TBB)
	set(preferred_allocator TBB)
else()
	list(APPEND possible_allocators BoostPool STL)
	set(preferred_allocator STL)
endif()
if(GOOGLE_PERFTOOLS_FOUND)
	list(APPEND possible_allocators tcmalloc)
endif()

# Check if current allocator choice is valid and set cache option
list(FIND possible_allocators "${GTSAM_DEFAULT_ALLOCATOR}" allocator_valid)
if(allocator_valid EQUAL -1)
	set(GTSAM_DEFAULT_ALLOCATOR ${preferred_allocator} CACHE STRING "Default allocator" FORCE)
else()
	set(GTSAM_DEFAULT_ALLOCATOR ${preferred_allocator} CACHE STRING "Default allocator")
endif()
set_property(CACHE GTSAM_DEFAULT_ALLOCATOR PROPERTY STRINGS ${possible_allocators})
mark_as_advanced(GTSAM_DEFAULT_ALLOCATOR)

# Define compile flags depending on allocator
if("${GTSAM_DEFAULT_ALLOCATOR}" STREQUAL "BoostPool")
	set(GTSAM_ALLOCATOR_BOOSTPOOL 1)
elseif("${GTSAM_DEFAULT_ALLOCATOR}" STREQUAL "STL")
	set(GTSAM_ALLOCATOR_STL 1)
elseif("${GTSAM_DEFAULT_ALLOCATOR}" STREQUAL "TBB")
	set(GTSAM_ALLOCATOR_TBB 1)
elseif("${GTSAM_DEFAULT_ALLOCATOR}" STREQUAL "tcmalloc")
	set(GTSAM_ALLOCATOR_STL 1) # tcmalloc replaces malloc, so to use it we use the STL allocator
	list(APPEND GTSAM_ADDITIONAL_LIBRARIES "tcmalloc")
endif()

if(MSVC)
	list_append_cache(GTSAM_COMPILE_DEFINITIONS_PRIVATE _CRT_SECURE_NO_WARNINGS _SCL_SECURE_NO_WARNINGS)
	list_append_cache(GTSAM_COMPILE_OPTIONS_PRIVATE /wd4251 /wd4275 /wd4251 /wd4661 /wd4344 /wd4503) # Disable non-DLL-exported base class and other warnings
	list_append_cache(GTSAM_COMPILE_OPTIONS_PRIVATE /bigobj) # Allow large object files for template-based code
endif()

# GCC 4.8+ complains about local typedefs which we use for shared_ptr etc.
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  if (NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS 4.8)
    list_append_cache(GTSAM_COMPILE_OPTIONS_PRIVATE -Wno-unused-local-typedefs)
  endif()
endif()

# As of XCode 7, clang also complains about this
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
  if (NOT CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7.0)
    list_append_cache(GTSAM_COMPILE_OPTIONS_PRIVATE -Wno-unused-local-typedefs)
  endif()
endif()

if(GTSAM_ENABLE_CONSISTENCY_CHECKS)
  # This should be made PUBLIC if GTSAM_EXTRA_CONSISTENCY_CHECKS is someday used in a public .h
  list_append_cache(GTSAM_COMPILE_DEFINITIONS_PRIVATE GTSAM_EXTRA_CONSISTENCY_CHECKS)
endif()

###############################################################################
# Add components

# Build CppUnitLite
add_subdirectory(CppUnitLite)

# Build wrap
if (GTSAM_BUILD_WRAP)
    add_subdirectory(wrap)
	# suppress warning of cython line being too long
	if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
		set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-misleading-indentation")
	endif()
endif(GTSAM_BUILD_WRAP)

# Build GTSAM library
add_subdirectory(gtsam)

# Build Tests
add_subdirectory(tests)

# Build examples
add_subdirectory(examples)

# Build timing
add_subdirectory(timing)

# Build gtsam_unstable
if (GTSAM_BUILD_UNSTABLE)
    add_subdirectory(gtsam_unstable)
endif()

# Matlab toolbox
if (GTSAM_INSTALL_MATLAB_TOOLBOX)
	add_subdirectory(matlab)
endif()

# Cython wrap
if (GTSAM_INSTALL_CYTHON_TOOLBOX)
  set(GTSAM_INSTALL_CYTHON_TOOLBOX 1)
  # Set up cache options
  # Cython install path appended with Build type (e.g. cython, cythonDebug, etc).
  # This does not override custom values set from the command line
  set(GTSAM_CYTHON_INSTALL_PATH "${PROJECT_BINARY_DIR}/cython${GTSAM_BUILD_TAG}" CACHE PATH "Cython toolbox destination, blank defaults to PROJECT_BINARY_DIR/cython<GTSAM_BUILD_TAG>")
  set(GTSAM_EIGENCY_INSTALL_PATH ${GTSAM_CYTHON_INSTALL_PATH}/gtsam_eigency)
  add_subdirectory(cython ${GTSAM_CYTHON_INSTALL_PATH})
else()
  set(GTSAM_INSTALL_CYTHON_TOOLBOX 0) # This will go into config.h
endif()


# Install config and export files
GtsamMakeConfigFile(GTSAM "${CMAKE_CURRENT_SOURCE_DIR}/gtsam_extra.cmake.in")
export(TARGETS ${GTSAM_EXPORTED_TARGETS} FILE GTSAM-exports.cmake)


# Check for doxygen availability - optional dependency
find_package(Doxygen)

# Doxygen documentation - enabling options in subfolder
if (DOXYGEN_FOUND)
    add_subdirectory(doc)
endif()

# CMake Tools
add_subdirectory(cmake)


###############################################################################
# Set up CPack
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "GTSAM")
set(CPACK_PACKAGE_VENDOR "Frank Dellaert, Georgia Institute of Technology")
set(CPACK_PACKAGE_CONTACT "Frank Dellaert, dellaert@cc.gatech.edu")
set(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/README.md")
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


###############################################################################
# Print configuration variables
message(STATUS "===============================================================")
message(STATUS "================  Configuration Options  ======================")
message(STATUS "  CMAKE_CXX_COMPILER_ID type     : ${CMAKE_CXX_COMPILER_ID}")
message(STATUS "  CMAKE_CXX_COMPILER_VERSION     : ${CMAKE_CXX_COMPILER_VERSION}")
message(STATUS "  CMake version                  : ${CMAKE_VERSION}")
message(STATUS "  CMake generator                : ${CMAKE_GENERATOR}")
message(STATUS "  CMake build tool               : ${CMAKE_BUILD_TOOL}")
message(STATUS "Build flags                                               ")
print_config_flag(${GTSAM_BUILD_TESTS}                 "Build Tests                    ")
print_config_flag(${GTSAM_BUILD_EXAMPLES_ALWAYS}       "Build examples with 'make all' ")
print_config_flag(${GTSAM_BUILD_TIMING_ALWAYS}         "Build timing scripts with 'make all'")
if (DOXYGEN_FOUND)
    print_config_flag(${GTSAM_BUILD_DOCS}              "Build Docs                     ")
endif()
print_config_flag(${BUILD_SHARED_LIBS}                 "Build shared GTSAM libraries   ")
print_config_flag(${GTSAM_BUILD_TYPE_POSTFIXES}        "Put build type in library name ")
if(GTSAM_UNSTABLE_AVAILABLE)
    print_config_flag(${GTSAM_BUILD_UNSTABLE}          "Build libgtsam_unstable        ")
endif()

if(NOT MSVC AND NOT XCODE_VERSION)
    print_config_flag(${GTSAM_BUILD_WITH_MARCH_NATIVE}     "Build for native architecture  ")
    message(STATUS "  Build type                     : ${CMAKE_BUILD_TYPE}")
    message(STATUS "  C compilation flags            : ${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_${CMAKE_BUILD_TYPE_UPPER}}")
    message(STATUS "  C++ compilation flags          : ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${CMAKE_BUILD_TYPE_UPPER}}")
endif()

print_build_options_for_target(gtsam)

message(STATUS "  Use System Eigen                : ${GTSAM_USE_SYSTEM_EIGEN} (Using version: ${GTSAM_EIGEN_VERSION})")

if(GTSAM_USE_TBB)
	message(STATUS "  Use Intel TBB                   : Yes")
elseif(TBB_FOUND)
	message(STATUS "  Use Intel TBB                   : TBB found but GTSAM_WITH_TBB is disabled")
else()
	message(STATUS "  Use Intel TBB                   : TBB not found")
endif()
if(GTSAM_USE_EIGEN_MKL)
	message(STATUS "  Eigen will use MKL              : Yes")
elseif(MKL_FOUND)
	message(STATUS "  Eigen will use MKL              : MKL found but GTSAM_WITH_EIGEN_MKL is disabled")
else()
	message(STATUS "  Eigen will use MKL              : MKL not found")
endif()
if(GTSAM_USE_EIGEN_MKL_OPENMP)
	message(STATUS "  Eigen will use MKL and OpenMP   : Yes")
elseif(OPENMP_FOUND AND NOT GTSAM_WITH_EIGEN_MKL)
	message(STATUS "  Eigen will use MKL and OpenMP   : OpenMP found but GTSAM_WITH_EIGEN_MKL is disabled")
elseif(OPENMP_FOUND AND NOT MKL_FOUND)
	message(STATUS "  Eigen will use MKL and OpenMP   : OpenMP found but MKL not found")
elseif(OPENMP_FOUND)
	message(STATUS "  Eigen will use MKL and OpenMP   : OpenMP found but GTSAM_WITH_EIGEN_MKL_OPENMP is disabled")
else()
	message(STATUS "  Eigen will use MKL and OpenMP   : OpenMP not found")
endif()
message(STATUS "  Default allocator               : ${GTSAM_DEFAULT_ALLOCATOR}")

if(GTSAM_THROW_CHEIRALITY_EXCEPTION)
	message(STATUS "  Cheirality exceptions enabled   : YES")
else()
	message(STATUS "  Cheirality exceptions enabled   : NO")
endif()

if(NOT MSVC AND NOT XCODE_VERSION)
	if(CCACHE_FOUND AND GTSAM_BUILD_WITH_CCACHE)
		message(STATUS "  Build with ccache               : Yes")
	elseif(CCACHE_FOUND)
		message(STATUS "  Build with ccache               : ccache found but GTSAM_BUILD_WITH_CCACHE is disabled")
	else()
		message(STATUS "  Build with ccache               : No")
	endif()
endif()

message(STATUS "Packaging flags                                               ")
message(STATUS "  CPack Source Generator          : ${CPACK_SOURCE_GENERATOR}")
message(STATUS "  CPack Generator                 : ${CPACK_GENERATOR}")

message(STATUS "GTSAM flags                                               ")
print_config_flag(${GTSAM_USE_QUATERNIONS}             "Quaternions as default Rot3     ")
print_config_flag(${GTSAM_ENABLE_CONSISTENCY_CHECKS}   "Runtime consistency checking    ")
print_config_flag(${GTSAM_ROT3_EXPMAP}                 "Rot3 retract is full ExpMap     ")
print_config_flag(${GTSAM_POSE3_EXPMAP}                "Pose3 retract is full ExpMap    ")
print_config_flag(${GTSAM_ALLOW_DEPRECATED_SINCE_V4}   "Deprecated in GTSAM 4 allowed   ")
print_config_flag(${GTSAM_TYPEDEF_POINTS_TO_VECTORS}   "Point3 is typedef to Vector3    ")
print_config_flag(${GTSAM_SUPPORT_NESTED_DISSECTION}   "Metis-based Nested Dissection   ")
print_config_flag(${GTSAM_TANGENT_PREINTEGRATION}      "Use tangent-space preintegration")
print_config_flag(${GTSAM_BUILD_WRAP}                  "Build Wrap                      ")

message(STATUS "MATLAB toolbox flags                                      ")
print_config_flag(${GTSAM_INSTALL_MATLAB_TOOLBOX}      "Install MATLAB toolbox          ")
if (${GTSAM_INSTALL_MATLAB_TOOLBOX})
    message(STATUS "  MATLAB root                     : ${MATLAB_ROOT}")
    message(STATUS "  MEX binary                      : ${MEX_COMMAND}")
endif()

message(STATUS "Cython toolbox flags                                      ")
print_config_flag(${GTSAM_INSTALL_CYTHON_TOOLBOX}      "Install Cython toolbox          ")
if(GTSAM_INSTALL_CYTHON_TOOLBOX)
	message(STATUS "  Python version                  : ${GTSAM_PYTHON_VERSION}")
endif()
message(STATUS "===============================================================")

# Print warnings at the end
if(GTSAM_WITH_TBB AND NOT TBB_FOUND)
	message(WARNING "TBB 4.4 or newer was not found - this is ok, but note that GTSAM parallelization will be disabled.  Set GTSAM_WITH_TBB to 'Off' to avoid this warning.")
endif()
if(GTSAM_WITH_EIGEN_MKL AND NOT MKL_FOUND)
	message(WARNING "MKL was not found - this is ok, but note that MKL will be disabled.  Set GTSAM_WITH_EIGEN_MKL to 'Off' to disable this warning.  See INSTALL.md for notes on performance.")
endif()
if(GTSAM_WITH_EIGEN_MKL_OPENMP AND NOT OPENMP_FOUND AND MKL_FOUND)
	message(WARNING "Your compiler does not support OpenMP.  Set GTSAM_WITH_EIGEN_MKL_OPENMP to 'Off' to avoid this warning. See INSTALL.md for notes on performance.")
endif()

# Include CPack *after* all flags
include(CPack)
