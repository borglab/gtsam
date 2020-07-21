# Set cmake policy to recognize the AppleClang compiler
# independently from the Clang compiler.
if(POLICY CMP0025)
  cmake_policy(SET CMP0025 NEW)
endif()

# function:  list_append_cache(var [new_values ...])
# Like "list(APPEND ...)" but working for CACHE variables.
# -----------------------------------------------------------
function(list_append_cache var)
  set(cur_value ${${var}})
  list(APPEND cur_value ${ARGN})
  get_property(MYVAR_DOCSTRING CACHE ${var} PROPERTY HELPSTRING)
  set(${var} "${cur_value}" CACHE STRING "${MYVAR_DOCSTRING}" FORCE)
endfunction()

# function:  append_config_if_not_empty(TARGET_VARIABLE build_type)
# Auxiliary function used to merge configuration-specific flags into the
# global variables that will actually be send to cmake targets.
# -----------------------------------------------------------
function(append_config_if_not_empty TARGET_VARIABLE_ build_type)
  string(TOUPPER "${build_type}" build_type_toupper)
  set(flags_variable_name "${TARGET_VARIABLE_}_${build_type_toupper}")
  set(flags_ ${${flags_variable_name}})
  if (NOT "${flags_}" STREQUAL "")
    if (${build_type_toupper} STREQUAL "COMMON")
      # Special "COMMON" configuration type, just append without CMake expression:
      list_append_cache(${TARGET_VARIABLE_} "${flags_}")
    else()
      # Regular configuration type:
      list_append_cache(${TARGET_VARIABLE_} "$<$<CONFIG:${build_type}>:${flags_}>")
    endif()
  endif()
endfunction()


# Add install prefix to search path
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}")


# Set up build types for MSVC and XCode
set(GTSAM_CMAKE_CONFIGURATION_TYPES Debug Release Timing Profiling RelWithDebInfo MinSizeRel
      CACHE STRING "Build types available to MSVC and XCode")
mark_as_advanced(FORCE GTSAM_CMAKE_CONFIGURATION_TYPES)
set(CMAKE_CONFIGURATION_TYPES ${GTSAM_CMAKE_CONFIGURATION_TYPES} CACHE STRING "Build configurations" FORCE)


# Default to Release mode
if(NOT CMAKE_BUILD_TYPE AND NOT MSVC AND NOT XCODE_VERSION)
    set(GTSAM_CMAKE_BUILD_TYPE "Release" CACHE STRING
      "Choose the type of build, options are: None Debug Release Timing Profiling RelWithDebInfo.")
    set(CMAKE_BUILD_TYPE ${GTSAM_CMAKE_BUILD_TYPE} CACHE STRING
      "Choose the type of build, options are: None Debug Release Timing Profiling RelWithDebInfo." FORCE)
endif()

# Add option for using build type postfixes to allow installing multiple build modes
option(GTSAM_BUILD_TYPE_POSTFIXES        "Enable/Disable appending the build type to the name of compiled libraries" ON)

# Define all cache variables, to be populated below depending on the OS/compiler:
set(GTSAM_COMPILE_OPTIONS_PRIVATE        "" CACHE STRING "(Do not edit) Private compiler flags for all build configurations." FORCE)
set(GTSAM_COMPILE_OPTIONS_PUBLIC         "" CACHE STRING "(Do not edit) Public compiler flags (exported to user projects) for all build configurations."  FORCE)
set(GTSAM_COMPILE_DEFINITIONS_PRIVATE    "" CACHE STRING "(Do not edit) Private preprocessor macros for all build configurations." FORCE)
set(GTSAM_COMPILE_DEFINITIONS_PUBLIC     "" CACHE STRING "(Do not edit) Public preprocessor macros for all build configurations." FORCE)
mark_as_advanced(GTSAM_COMPILE_OPTIONS_PRIVATE)
mark_as_advanced(GTSAM_COMPILE_OPTIONS_PUBLIC)
mark_as_advanced(GTSAM_COMPILE_DEFINITIONS_PRIVATE)
mark_as_advanced(GTSAM_COMPILE_DEFINITIONS_PUBLIC)

foreach(build_type ${GTSAM_CMAKE_CONFIGURATION_TYPES})
  string(TOUPPER "${build_type}" build_type_toupper)

  # Define empty cache variables for "public". "private" are creaed below.
  set(GTSAM_COMPILE_OPTIONS_PUBLIC_${build_type_toupper}      "" CACHE STRING "(User editable) Public compiler flags (exported to user projects) for `${build_type_toupper}` configuration.")
  set(GTSAM_COMPILE_DEFINITIONS_PUBLIC_${build_type_toupper}  "" CACHE STRING "(User editable) Public preprocessor macros for `${build_type_toupper}` configuration.")
endforeach()

# Common preprocessor macros for each configuration:
set(GTSAM_COMPILE_DEFINITIONS_PRIVATE_DEBUG           "_DEBUG;EIGEN_INITIALIZE_MATRICES_BY_NAN" CACHE STRING "(User editable) Private preprocessor macros for Debug configuration.")
set(GTSAM_COMPILE_DEFINITIONS_PRIVATE_RELWITHDEBINFO  "NDEBUG" CACHE STRING "(User editable) Private preprocessor macros for RelWithDebInfo configuration.")
set(GTSAM_COMPILE_DEFINITIONS_PRIVATE_RELEASE         "NDEBUG" CACHE STRING "(User editable) Private preprocessor macros for Release configuration.")
set(GTSAM_COMPILE_DEFINITIONS_PRIVATE_PROFILING       "NDEBUG" CACHE STRING "(User editable) Private preprocessor macros for Profiling configuration.")
set(GTSAM_COMPILE_DEFINITIONS_PRIVATE_TIMING          "NDEBUG;ENABLE_TIMING" CACHE STRING "(User editable) Private preprocessor macros for Timing configuration.")
if(MSVC)
  # Common to all configurations:
  list_append_cache(GTSAM_COMPILE_DEFINITIONS_PRIVATE
    WINDOWS_LEAN_AND_MEAN
    NOMINMAX
	)
  # Avoid literally hundreds to thousands of warnings:
  list_append_cache(GTSAM_COMPILE_OPTIONS_PUBLIC
	/wd4267 # warning C4267: 'initializing': conversion from 'size_t' to 'int', possible loss of data
  )

endif()

# Other (non-preprocessor macros) compiler flags:
if(MSVC)
  # Common to all configurations, next for each configuration:
  set(GTSAM_COMPILE_OPTIONS_PRIVATE_COMMON          /W3 /GR /EHsc /MP  CACHE STRING "(User editable) Private compiler flags for all configurations.")
  set(GTSAM_COMPILE_OPTIONS_PRIVATE_DEBUG           /MDd /Zi /Ob0 /Od /RTC1  CACHE STRING "(User editable) Private compiler flags for Debug configuration.")
  set(GTSAM_COMPILE_OPTIONS_PRIVATE_RELWITHDEBINFO  /MD /O2 /D /Zi /d2Zi+  CACHE STRING "(User editable) Private compiler flags for RelWithDebInfo configuration.")
  set(GTSAM_COMPILE_OPTIONS_PRIVATE_RELEASE         /MD /O2  CACHE STRING "(User editable) Private compiler flags for Release configuration.")
  set(GTSAM_COMPILE_OPTIONS_PRIVATE_PROFILING       /MD /O2  /Zi  CACHE STRING "(User editable) Private compiler flags for Profiling configuration.")
  set(GTSAM_COMPILE_OPTIONS_PRIVATE_TIMING          /MD /O2  CACHE STRING "(User editable) Private compiler flags for Timing configuration.")
else()
  # Common to all configurations, next for each configuration:
  # "-fPIC" is to ensure proper code generation for shared libraries
  set(GTSAM_COMPILE_OPTIONS_PRIVATE_COMMON          -Wall -fPIC CACHE STRING "(User editable) Private compiler flags for all configurations.")
  set(GTSAM_COMPILE_OPTIONS_PRIVATE_DEBUG           -g -fno-inline  CACHE STRING "(User editable) Private compiler flags for Debug configuration.")
  set(GTSAM_COMPILE_OPTIONS_PRIVATE_RELWITHDEBINFO  -g -O3  CACHE STRING "(User editable) Private compiler flags for RelWithDebInfo configuration.")
  set(GTSAM_COMPILE_OPTIONS_PRIVATE_RELEASE         -O3  CACHE STRING "(User editable) Private compiler flags for Release configuration.")
  set(GTSAM_COMPILE_OPTIONS_PRIVATE_PROFILING       -O3  CACHE STRING "(User editable) Private compiler flags for Profiling configuration.")
  set(GTSAM_COMPILE_OPTIONS_PRIVATE_TIMING          -g -O3  CACHE STRING "(User editable) Private compiler flags for Timing configuration.")
endif()

# Enable C++11:
if (NOT CMAKE_VERSION VERSION_LESS 3.8)
    set(GTSAM_COMPILE_FEATURES_PUBLIC "cxx_std_11" CACHE STRING "CMake compile features property for all gtsam targets.")
    # See: https://cmake.org/cmake/help/latest/prop_tgt/CXX_EXTENSIONS.html
    # This is to enable -std=c++11 instead of -std=g++11
    set(CMAKE_CXX_EXTENSIONS OFF)
else()
  # Old cmake versions:
  if (NOT MSVC)
    list_append_cache(GTSAM_COMPILE_OPTIONS_PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-std=c++11>)
  endif()
endif()

# Merge all user-defined flags into the variables that are to be actually used by CMake:
foreach(build_type "common" ${GTSAM_CMAKE_CONFIGURATION_TYPES})
  append_config_if_not_empty(GTSAM_COMPILE_OPTIONS_PRIVATE ${build_type})
  append_config_if_not_empty(GTSAM_COMPILE_OPTIONS_PUBLIC  ${build_type})
  append_config_if_not_empty(GTSAM_COMPILE_DEFINITIONS_PRIVATE  ${build_type})
  append_config_if_not_empty(GTSAM_COMPILE_DEFINITIONS_PUBLIC   ${build_type})
endforeach()

# Linker flags:
set(GTSAM_CMAKE_SHARED_LINKER_FLAGS_TIMING  "${CMAKE_SHARED_LINKER_FLAGS_RELEASE}" CACHE STRING "Linker flags during timing builds.")
set(GTSAM_CMAKE_MODULE_LINKER_FLAGS_TIMING  "${CMAKE_MODULE_LINKER_FLAGS_RELEASE}" CACHE STRING "Linker flags during timing builds.")
set(GTSAM_CMAKE_EXE_LINKER_FLAGS_TIMING     "${CMAKE_EXE_LINKER_FLAGS_RELEASE}" CACHE STRING "Linker flags during timing builds.")

set(GTSAM_CMAKE_SHARED_LINKER_FLAGS_PROFILING "${CMAKE_SHARED_LINKER_FLAGS_RELEASE}" CACHE STRING "Linker flags during profiling builds.")
set(GTSAM_CMAKE_MODULE_LINKER_FLAGS_PROFILING "${CMAKE_MODULE_LINKER_FLAGS_RELEASE}" CACHE STRING "Linker flags during profiling builds.")
set(GTSAM_CMAKE_EXE_LINKER_FLAGS_PROFILING    "${CMAKE_EXE_LINKER_FLAGS_RELEASE}" CACHE STRING "Linker flags during profiling builds.")

mark_as_advanced(GTSAM_CMAKE_EXE_LINKER_FLAGS_TIMING
                 GTSAM_CMAKE_SHARED_LINKER_FLAGS_TIMING GTSAM_CMAKE_MODULE_LINKER_FLAGS_TIMING
                 GTSAM_CMAKE_C_FLAGS_PROFILING GTSAM_ GTSAM_CMAKE_EXE_LINKER_FLAGS_PROFILING
                 GTSAM_CMAKE_SHARED_LINKER_FLAGS_PROFILING GTSAM_CMAKE_MODULE_LINKER_FLAGS_PROFILING)

set(CMAKE_SHARED_LINKER_FLAGS_TIMING ${GTSAM_CMAKE_SHARED_LINKER_FLAGS_TIMING})
set(CMAKE_MODULE_LINKER_FLAGS_TIMING ${GTSAM_CMAKE_MODULE_LINKER_FLAGS_TIMING})
set(CMAKE_EXE_LINKER_FLAGS_TIMING ${GTSAM_CMAKE_EXE_LINKER_FLAGS_TIMING})

set(CMAKE_SHARED_LINKER_FLAGS_PROFILING ${GTSAM_CMAKE_SHARED_LINKER_FLAGS_PROFILING})
set(CMAKE_MODULE_LINKER_FLAGS_PROFILING ${GTSAM_CMAKE_MODULE_LINKER_FLAGS_PROFILING})
set(CMAKE_EXE_LINKER_FLAGS_PROFILING ${GTSAM_CMAKE_EXE_LINKER_FLAGS_PROFILING})

# Clang uses a template depth that is less than standard and is too small
if(${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang")
    # Apple Clang before 5.0 does not support -ftemplate-depth.
    if(NOT (APPLE AND "${CMAKE_CXX_COMPILER_VERSION}" VERSION_LESS "5.0"))
        list_append_cache(GTSAM_COMPILE_OPTIONS_PUBLIC "-ftemplate-depth=1024")
    endif()
endif()

if (NOT MSVC)
  option(GTSAM_BUILD_WITH_MARCH_NATIVE  "Enable/Disable building with all instructions supported by native architecture (binary may not be portable!)" ON)
  if(GTSAM_BUILD_WITH_MARCH_NATIVE)
    # Add as public flag so all dependant projects also use it, as required
    # by Eigen to avid crashes due to SIMD vectorization:
    list_append_cache(GTSAM_COMPILE_OPTIONS_PUBLIC "-march=native")
  endif()
endif()

# Set up build type library postfixes
if(GTSAM_BUILD_TYPE_POSTFIXES)
  foreach(build_type Debug Timing Profiling RelWithDebInfo MinSizeRel)
    string(TOUPPER "${build_type}" build_type_toupper)
    set(CMAKE_${build_type_toupper}_POSTFIX ${build_type})
  endforeach()
endif()

# Make common binary output directory when on Windows
if(WIN32)
  set(RUNTIME_OUTPUT_PATH "${CMAKE_BINARY_DIR}/bin")
  set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/bin")
  set(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}/lib")
endif()

# Set up build type list for cmake-gui
if(NOT "${CMAKE_BUILD_TYPE}" STREQUAL "")
  if(${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} VERSION_GREATER 2.8 OR ${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} VERSION_EQUAL 2.8)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS None Debug Release Timing Profiling RelWithDebInfo MinSizeRel)
  endif()
endif()

# Check build types
string(TOLOWER "${CMAKE_BUILD_TYPE}" cmake_build_type_tolower)
if(    NOT cmake_build_type_tolower STREQUAL ""
   AND NOT cmake_build_type_tolower STREQUAL "none"
   AND NOT cmake_build_type_tolower STREQUAL "debug"
   AND NOT cmake_build_type_tolower STREQUAL "release"
   AND NOT cmake_build_type_tolower STREQUAL "timing"
   AND NOT cmake_build_type_tolower STREQUAL "profiling"
   AND NOT cmake_build_type_tolower STREQUAL "relwithdebinfo"
   AND NOT cmake_build_type_tolower STREQUAL "minsizerel")
  message(FATAL_ERROR "Unknown build type \"${CMAKE_BUILD_TYPE}\". Allowed values are None, Debug, Release, Timing, Profiling, RelWithDebInfo, MinSizeRel (case-insensitive).")
endif()

# Enable Visual Studio solution folders
set_property(GLOBAL PROPERTY USE_FOLDERS On)

# Function for automatically assigning source folders
function(gtsam_assign_source_folders)
  set(FILES ${ARGV})
  foreach(file ${FILES})
    file(RELATIVE_PATH relative_file "${CMAKE_CURRENT_SOURCE_DIR}" "${file}")
    get_filename_component(relative_path "${relative_file}" PATH)
    file(TO_NATIVE_PATH "${relative_path}" relative_path)
    source_group("${relative_path}" FILES "${file}")
  endforeach()
endfunction()

# Find and assign all source and header files
function(gtsam_assign_all_source_folders)
  file(GLOB_RECURSE all_c_srcs "*.c")
  file(GLOB_RECURSE all_cpp_srcs "*.cpp")
  file(GLOB_RECURSE all_headers "*.h")
  gtsam_assign_source_folders("${all_c_srcs};${all_cpp_srcs};${all_headers}")
endfunction()

# Applies the per-config build flags to the given target (e.g. gtsam, wrap_lib)
function(gtsam_apply_build_flags target_name_)
  # To enable C++11: the use of target_compile_features() is preferred since
  # it will be not in conflict with a more modern C++ standard, if used in a
  # client program.
  if (NOT "${GTSAM_COMPILE_FEATURES_PUBLIC}" STREQUAL "")
  	target_compile_features(${target_name_} PUBLIC ${GTSAM_COMPILE_FEATURES_PUBLIC})
  endif()

  target_compile_definitions(${target_name_} PRIVATE ${GTSAM_COMPILE_DEFINITIONS_PRIVATE})
  target_compile_definitions(${target_name_} PUBLIC ${GTSAM_COMPILE_DEFINITIONS_PUBLIC})
  if (NOT "${GTSAM_COMPILE_OPTIONS_PUBLIC}" STREQUAL "")
    target_compile_options(${target_name_} PUBLIC ${GTSAM_COMPILE_OPTIONS_PUBLIC})
  endif()
  target_compile_options(${target_name_} PRIVATE ${GTSAM_COMPILE_OPTIONS_PRIVATE})

endfunction(gtsam_apply_build_flags)
