
# Add install prefix to search path
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}")

# Default to Release mode
if(NOT CMAKE_BUILD_TYPE AND NOT MSVC AND NOT XCODE_VERSION)
    set(GTSAM_CMAKE_BUILD_TYPE "Release" CACHE STRING
      "Choose the type of build, options are: None Debug Release Timing Profiling RelWithDebInfo.")
    set(CMAKE_BUILD_TYPE ${GTSAM_CMAKE_BUILD_TYPE} CACHE STRING
      "Choose the type of build, options are: None Debug Release Timing Profiling RelWithDebInfo." FORCE)
endif()

# Add option for using build type postfixes to allow installing multiple build modes
option(GTSAM_BUILD_TYPE_POSTFIXES        "Enable/Disable appending the build type to the name of compiled libraries" ON)

# Set custom compilation flags.
# NOTE: We set all the CACHE variables with a GTSAM prefix, and then set a normal local variable below
# so that we don't "pollute" the global variable namespace in the cmake cache.
  # Set all CMAKE_BUILD_TYPE flags:
  # (see https://cmake.org/Wiki/CMake_Useful_Variables#Compilers_and_Tools)
if(MSVC)
    set(GTSAM_CMAKE_C_FLAGS                  "/W3 /GR /EHsc /MP /DWINDOWS_LEAN_AND_MEAN" CACHE STRING "Flags used by the compiler for all builds.")
    set(GTSAM_CMAKE_CXX_FLAGS                "/W3 /GR /EHsc /MP /DWINDOWS_LEAN_AND_MEAN" CACHE STRING "Flags used by the compiler for all builds.")
    set(GTSAM_CMAKE_C_FLAGS_DEBUG            "/D_DEBUG /MDd /Zi /Ob0 /Od /RTC1 /DEIGEN_INITIALIZE_MATRICES_BY_NAN" CACHE STRING "Extra flags used by the compiler during debug builds.")
    set(GTSAM_CMAKE_CXX_FLAGS_DEBUG          "/D_DEBUG /MDd /Zi /Ob0 /Od /RTC1 /DEIGEN_INITIALIZE_MATRICES_BY_NAN" CACHE STRING "Extra flags used by the compiler during debug builds.")
    set(GTSAM_CMAKE_C_FLAGS_RELWITHDEBINFO   "/MD /O2 /DNDEBUG /Zi /d2Zi+" CACHE STRING "Extra flags used by the compiler during relwithdebinfo builds.")
    set(GTSAM_CMAKE_CXX_FLAGS_RELWITHDEBINFO "/MD /O2 /DNDEBUG /Zi /d2Zi+" CACHE STRING "Extra flags used by the compiler during relwithdebinfo builds.")
    set(GTSAM_CMAKE_C_FLAGS_RELEASE          "/MD /O2 /DNDEBUG" CACHE STRING "Extra flags used by the compiler during release builds.")
    set(GTSAM_CMAKE_CXX_FLAGS_RELEASE        "/MD /O2 /DNDEBUG" CACHE STRING "Extra flags used by the compiler during release builds.")
    set(GTSAM_CMAKE_C_FLAGS_PROFILING        "${GTSAM_CMAKE_C_FLAGS_RELEASE}   /Zi" CACHE STRING "Extra flags used by the compiler during profiling builds.")
    set(GTSAM_CMAKE_CXX_FLAGS_PROFILING      "${GTSAM_CMAKE_CXX_FLAGS_RELEASE} /Zi" CACHE STRING "Extra flags used by the compiler during profiling builds.")
    set(GTSAM_CMAKE_C_FLAGS_TIMING           "${GTSAM_CMAKE_C_FLAGS_RELEASE}   /DENABLE_TIMING" CACHE STRING "Extra flags used by the compiler during timing builds.")
    set(GTSAM_CMAKE_CXX_FLAGS_TIMING         "${GTSAM_CMAKE_CXX_FLAGS_RELEASE} /DENABLE_TIMING" CACHE STRING "Extra flags used by the compiler during timing builds.")
else()
    set(GTSAM_CMAKE_C_FLAGS                  "-std=c11   -Wall" CACHE STRING "Flags used by the compiler for all builds.")
    set(GTSAM_CMAKE_CXX_FLAGS                "-std=c++11 -Wall" CACHE STRING "Flags used by the compiler for all builds.")
    set(GTSAM_CMAKE_C_FLAGS_DEBUG            "-g -fno-inline -DEIGEN_INITIALIZE_MATRICES_BY_NAN" CACHE STRING "Extra flags used by the compiler during debug builds.")
    set(GTSAM_CMAKE_CXX_FLAGS_DEBUG          "-g -fno-inline -DEIGEN_INITIALIZE_MATRICES_BY_NAN" CACHE STRING "Extra flags used by the compiler during debug builds.")
    set(GTSAM_CMAKE_C_FLAGS_RELWITHDEBINFO   "-g -O3 -DNDEBUG" CACHE STRING "Extra flags used by the compiler during relwithdebinfo builds.")
    set(GTSAM_CMAKE_CXX_FLAGS_RELWITHDEBINFO "-g -O3 -DNDEBUG" CACHE STRING "Extra flags used by the compiler during relwithdebinfo builds.")
    set(GTSAM_CMAKE_C_FLAGS_RELEASE          "   -O3 -DNDEBUG" CACHE STRING "Extra flags used by the compiler during release builds.")
    set(GTSAM_CMAKE_CXX_FLAGS_RELEASE        "   -O3 -DNDEBUG" CACHE STRING "Extra flags used by the compiler during release builds.")
    set(GTSAM_CMAKE_C_FLAGS_PROFILING        "${GTSAM_CMAKE_C_FLAGS_RELEASE}" CACHE STRING "Extra flags used by the compiler during profiling builds.")
    set(GTSAM_CMAKE_CXX_FLAGS_PROFILING      "${GTSAM_CMAKE_CXX_FLAGS_RELEASE}" CACHE STRING "Extra flags used by the compiler during profiling builds.")
    set(GTSAM_CMAKE_C_FLAGS_TIMING           "${GTSAM_CMAKE_C_FLAGS_RELEASE} -DENABLE_TIMING" CACHE STRING "Extra flags used by the compiler during timing builds.")
    set(GTSAM_CMAKE_CXX_FLAGS_TIMING         "${GTSAM_CMAKE_CXX_FLAGS_RELEASE} -DENABLE_TIMING" CACHE STRING "Extra flags used by the compiler during timing builds.")
endif()

set(GTSAM_CMAKE_SHARED_LINKER_FLAGS_TIMING  "${CMAKE_SHARED_LINKER_FLAGS_RELEASE}" CACHE STRING "Linker flags during timing builds.")
set(GTSAM_CMAKE_MODULE_LINKER_FLAGS_TIMING  "${CMAKE_MODULE_LINKER_FLAGS_RELEASE}" CACHE STRING "Linker flags during timing builds.")
set(GTSAM_CMAKE_EXE_LINKER_FLAGS_TIMING     "${CMAKE_EXE_LINKER_FLAGS_RELEASE}" CACHE STRING "Linker flags during timing builds.")

set(GTSAM_CMAKE_SHARED_LINKER_FLAGS_PROFILING "${CMAKE_SHARED_LINKER_FLAGS_RELEASE}" CACHE STRING "Linker flags during profiling builds.")
set(GTSAM_CMAKE_MODULE_LINKER_FLAGS_PROFILING "${CMAKE_MODULE_LINKER_FLAGS_RELEASE}" CACHE STRING "Linker flags during profiling builds.")
set(GTSAM_CMAKE_EXE_LINKER_FLAGS_PROFILING    "${CMAKE_EXE_LINKER_FLAGS_RELEASE}" CACHE STRING "Linker flags during profiling builds.")

mark_as_advanced(GTSAM_CMAKE_C_FLAGS_TIMING GTSAM_CMAKE_CXX_FLAGS_TIMING GTSAM_CMAKE_EXE_LINKER_FLAGS_TIMING 
                 GTSAM_CMAKE_SHARED_LINKER_FLAGS_TIMING GTSAM_CMAKE_MODULE_LINKER_FLAGS_TIMING
                 GTSAM_CMAKE_C_FLAGS_PROFILING GTSAM_CMAKE_CXX_FLAGS_PROFILING GTSAM_CMAKE_EXE_LINKER_FLAGS_PROFILING 
                 GTSAM_CMAKE_SHARED_LINKER_FLAGS_PROFILING GTSAM_CMAKE_MODULE_LINKER_FLAGS_PROFILING)

# Apply the gtsam specific build flags as normal variables. This makes it so that they only
# apply to the gtsam part of the build if gtsam is built as a subproject
set(CMAKE_C_FLAGS                  "${CMAKE_C_FLAGS} ${GTSAM_CMAKE_C_FLAGS}")
set(CMAKE_CXX_FLAGS                "${CMAKE_CXX_FLAGS} ${GTSAM_CMAKE_CXX_FLAGS}")
set(CMAKE_C_FLAGS_DEBUG            "${CMAKE_C_FLAGS_DEBUG} ${GTSAM_CMAKE_C_FLAGS_DEBUG}")
set(CMAKE_CXX_FLAGS_DEBUG          "${CMAKE_CXX_FLAGS_DEBUG} ${GTSAM_CMAKE_CXX_FLAGS_DEBUG}")
set(CMAKE_C_FLAGS_RELWITHDEBINFO   "${CMAKE_C_FLAGS_RELWITHDEBINFO} ${GTSAM_CMAKE_C_FLAGS_RELWITHDEBINFO}")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} ${GTSAM_CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
set(CMAKE_C_FLAGS_RELEASE          "${CMAKE_C_FLAGS_RELEASE} ${GTSAM_CMAKE_C_FLAGS_RELEASE}")
set(CMAKE_CXX_FLAGS_RELEASE        "${CMAKE_CXX_FLAGS_RELEASE} ${GTSAM_CMAKE_CXX_FLAGS_RELEASE}")
set(CMAKE_C_FLAGS_PROFILING        "${CMAKE_C_FLAGS_PROFILING} ${GTSAM_CMAKE_C_FLAGS_PROFILING}")
set(CMAKE_CXX_FLAGS_PROFILING      "${CMAKE_CXX_FLAGS_PROFILING} ${GTSAM_CMAKE_CXX_FLAGS_PROFILING}")
set(CMAKE_C_FLAGS_TIMING           "${CMAKE_C_FLAGS_TIMING} ${GTSAM_CMAKE_C_FLAGS_TIMING}")
set(CMAKE_CXX_FLAGS_TIMING         "${CMAKE_CXX_FLAGS_TIMING} ${GTSAM_CMAKE_CXX_FLAGS_TIMING}")

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
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftemplate-depth=1024")
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

# Set up build types for MSVC and XCode
set(GTSAM_CMAKE_CONFIGURATION_TYPES Debug Release Timing Profiling RelWithDebInfo MinSizeRel
      CACHE STRING "Build types available to MSVC and XCode")
mark_as_advanced(FORCE GTSAM_CMAKE_CONFIGURATION_TYPES)
set(CMAKE_CONFIGURATION_TYPES ${GTSAM_CMAKE_CONFIGURATION_TYPES})

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

