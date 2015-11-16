
# Add install prefix to search path
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}")

# Default to Release mode
if(NOT FIRST_PASS_DONE AND NOT CMAKE_BUILD_TYPE AND NOT MSVC AND NOT XCODE_VERSION)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING
      "Choose the type of build, options are: None Debug Release Timing Profiling RelWithDebInfo."
      FORCE)
endif()

# Add option for using build type postfixes to allow installing multiple build modes
option(GTSAM_BUILD_TYPE_POSTFIXES        "Enable/Disable appending the build type to the name of compiled libraries" ON)

# Add debugging flags but only on the first pass
if(NOT FIRST_PASS_DONE)
  if(MSVC)
    set(CMAKE_C_FLAGS_DEBUG            "/D_DEBUG /MDd /Zi /Ob0 /Od /RTC1 /W3 /GR /EHsc /MP /DWINDOWS_LEAN_AND_MEAN /DEIGEN_INITIALIZE_MATRICES_BY_NAN" CACHE STRING "Flags used by the compiler during debug builds." FORCE)
    set(CMAKE_CXX_FLAGS_DEBUG          "/D_DEBUG /MDd /Zi /Ob0 /Od /RTC1 /W3 /GR /EHsc /MP /DWINDOWS_LEAN_AND_MEAN /DEIGEN_INITIALIZE_MATRICES_BY_NAN" CACHE STRING "Flags used by the compiler during debug builds." FORCE)
    set(CMAKE_C_FLAGS_RELWITHDEBINFO   "/MD /O2 /DNDEBUG /W3 /GR /EHsc /MP /Zi /d2Zi+ /DWINDOWS_LEAN_AND_MEAN" CACHE STRING "Flags used by the compiler during relwithdebinfo builds." FORCE)
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "/MD /O2 /DNDEBUG /W3 /GR /EHsc /MP /Zi /d2Zi+ /DWINDOWS_LEAN_AND_MEAN" CACHE STRING "Flags used by the compiler during relwithdebinfo builds." FORCE)
    set(CMAKE_C_FLAGS_RELEASE          "/MD /O2 /DNDEBUG /W3 /GR /EHsc /MP /DWINDOWS_LEAN_AND_MEAN" CACHE STRING "Flags used by the compiler during release builds." FORCE)
    set(CMAKE_CXX_FLAGS_RELEASE        "/MD /O2 /DNDEBUG /W3 /GR /EHsc /MP /DWINDOWS_LEAN_AND_MEAN" CACHE STRING "Flags used by the compiler during release builds." FORCE)
    set(CMAKE_C_FLAGS_TIMING           "${CMAKE_C_FLAGS_RELEASE} /DENABLE_TIMING" CACHE STRING "Flags used by the compiler during timing builds." FORCE)
    set(CMAKE_CXX_FLAGS_TIMING         "${CMAKE_CXX_FLAGS_RELEASE} /DENABLE_TIMING" CACHE STRING "Flags used by the compiler during timing builds." FORCE)
    set(CMAKE_EXE_LINKER_FLAGS_TIMING  "${CMAKE_EXE_LINKER_FLAGS_RELEASE}" CACHE STRING "Linker flags during timing builds." FORCE)
    set(CMAKE_SHARED_LINKER_FLAGS_TIMING  "${CMAKE_SHARED_LINKER_FLAGS_RELEASE}" CACHE STRING "Linker flags during timing builds." FORCE)
    set(CMAKE_MODULE_LINKER_FLAGS_TIMING  "${CMAKE_MODULE_LINKER_FLAGS_RELEASE}" CACHE STRING "Linker flags during timing builds." FORCE)
    mark_as_advanced(CMAKE_C_FLAGS_TIMING CMAKE_CXX_FLAGS_TIMING CMAKE_EXE_LINKER_FLAGS_TIMING CMAKE_SHARED_LINKER_FLAGS_TIMING CMAKE_MODULE_LINKER_FLAGS_TIMING)
    set(CMAKE_C_FLAGS_PROFILING        "/MD /O2 /DNDEBUG /W3 /GR /EHsc /MP /Zi /DWINDOWS_LEAN_AND_MEAN" CACHE STRING "Flags used by the compiler during profiling builds." FORCE)
    set(CMAKE_CXX_FLAGS_PROFILING      "/MD /O2 /DNDEBUG /W3 /GR /EHsc /MP /Zi /DWINDOWS_LEAN_AND_MEAN" CACHE STRING "Flags used by the compiler during profiling builds." FORCE)
    set(CMAKE_EXE_LINKER_FLAGS_PROFILING "${CMAKE_EXE_LINKER_FLAGS_RELEASE}" CACHE STRING "Linker flags during profiling builds." FORCE)
    set(CMAKE_SHARED_LINKER_FLAGS_PROFILING "${CMAKE_SHARED_LINKER_FLAGS_RELEASE}" CACHE STRING "Linker flags during profiling builds." FORCE)
    set(CMAKE_MODULE_LINKER_FLAGS_PROFILING "${CMAKE_MODULE_LINKER_FLAGS_RELEASE}" CACHE STRING "Linker flags during profiling builds." FORCE)
    mark_as_advanced(CMAKE_C_FLAGS_PROFILING CMAKE_CXX_FLAGS_PROFILING CMAKE_EXE_LINKER_FLAGS_PROFILING CMAKE_SHARED_LINKER_FLAGS_PROFILING CMAKE_MODULE_LINKER_FLAGS_PROFILING)
  else()
    set(CMAKE_C_FLAGS_DEBUG            "${CMAKE_C_FLAGS_DEBUG} -std=c11 -fno-inline -Wall -DEIGEN_INITIALIZE_MATRICES_BY_NAN" CACHE STRING "Flags used by the compiler during debug builds." FORCE)
    set(CMAKE_CXX_FLAGS_DEBUG          "${CMAKE_CXX_FLAGS_DEBUG} -std=c++11 -fno-inline -Wall -DEIGEN_INITIALIZE_MATRICES_BY_NAN" CACHE STRING "Flags used by the compiler during debug builds." FORCE)
    set(CMAKE_C_FLAGS_RELWITHDEBINFO   "-std=c11 -g -O3 -Wall -DNDEBUG" CACHE STRING "Flags used by the compiler during relwithdebinfo builds." FORCE)
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-std=c++11 -g -O3 -Wall -DNDEBUG" CACHE STRING "Flags used by the compiler during relwithdebinfo builds." FORCE)
    set(CMAKE_C_FLAGS_RELEASE          "-std=c11 -O3 -Wall -DNDEBUG -Wall" CACHE STRING "Flags used by the compiler during release builds." FORCE)
    set(CMAKE_CXX_FLAGS_RELEASE        "-std=c++11 -O3 -Wall -DNDEBUG -Wall" CACHE STRING "Flags used by the compiler during release builds." FORCE)
    set(CMAKE_C_FLAGS_TIMING           "${CMAKE_C_FLAGS_RELEASE} -DENABLE_TIMING" CACHE STRING "Flags used by the compiler during timing builds." FORCE)
    set(CMAKE_CXX_FLAGS_TIMING         "${CMAKE_CXX_FLAGS_RELEASE} -DENABLE_TIMING" CACHE STRING "Flags used by the compiler during timing builds." FORCE)
    set(CMAKE_EXE_LINKER_FLAGS_TIMING  "${CMAKE_EXE_LINKER_FLAGS_RELEASE}" CACHE STRING "Linker flags during timing builds." FORCE)
    set(CMAKE_SHARED_LINKER_FLAGS_TIMING  "${CMAKE_EXE_LINKER_FLAGS_RELEASE}" CACHE STRING "Linker flags during timing builds." FORCE)
    mark_as_advanced(CMAKE_C_FLAGS_TIMING CMAKE_CXX_FLAGS_TIMING CMAKE_EXE_LINKER_FLAGS_TIMING CMAKE_SHARED_LINKER_FLAGS_TIMING)
    set(CMAKE_C_FLAGS_PROFILING        "-std=c11 -g -O3 -Wall -DNDEBUG" CACHE STRING "Flags used by the compiler during profiling builds." FORCE)
    set(CMAKE_CXX_FLAGS_PROFILING      "-std=c++11 -g -O3 -Wall -DNDEBUG" CACHE STRING "Flags used by the compiler during profiling builds." FORCE)
    set(CMAKE_EXE_LINKER_FLAGS_PROFILING "${CMAKE_EXE_LINKER_FLAGS_RELEASE}" CACHE STRING "Linker flags during profiling builds." FORCE)
	set(CMAKE_SHARED_LINKER_FLAGS_PROFILING "${CMAKE__LINKER_FLAGS_RELEASE}" CACHE STRING "Linker flags during profiling builds." FORCE)
    mark_as_advanced(CMAKE_C_FLAGS_PROFILING CMAKE_CXX_FLAGS_PROFILING CMAKE_EXE_LINKER_FLAGS_PROFILING CMAKE_SHARED_LINKER_FLAGS_PROFILING)
  endif()
endif()

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
if(NOT FIRST_PASS_DONE)
  set(CMAKE_CONFIGURATION_TYPES Debug Release Timing Profiling RelWithDebInfo MinSizeRel
      CACHE STRING "Build types available to MSVC and XCode" FORCE)
  mark_as_advanced(FORCE CMAKE_CONFIGURATION_TYPES)
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
  message(FATAL_ERROR "Unknown build type \"${CMAKE_BUILD_TYPE}\". Allowed values are None, Debug, Release, Timing, Profiling, RelWithDebInfo (case-insensitive).")
endif()

# Mark that first pass is done
set(FIRST_PASS_DONE TRUE CACHE INTERNAL "Internally used to mark whether cmake has been run multiple times")
mark_as_advanced(FIRST_PASS_DONE) 

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

