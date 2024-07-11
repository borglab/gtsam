project(gtsam LANGUAGES CXX)

# We split the library in to separate subfolders, each containing
# tests, timing, and an optional convenience library.
# The following variable is the master list of subdirs to add
set (gtsam_subdirs
    base
    basis
    geometry
    inference
    symbolic
    discrete
    hybrid
    linear
    nonlinear
    sam
    sfm
    slam
    navigation
)

set(gtsam_srcs)

# Build 3rdparty separately
message(STATUS "Building 3rdparty")
add_subdirectory(3rdparty)

set (3rdparty_srcs
 ${eigen_headers} # Set by 3rdparty/CMakeLists.txt
 ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/CCOLAMD/Source/ccolamd.c
 ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/SuiteSparse_config/SuiteSparse_config.c)
gtsam_assign_source_folders("${3rdparty_srcs}") # Create MSVC structure

# To exclude a source from the library build (in any subfolder)
# Add the full name to this list, as in the following example
# Sources to remove from builds
set (excluded_sources #"")
    "${CMAKE_CURRENT_SOURCE_DIR}/slam/serialization.cpp"
)

set (excluded_headers #"")
    "${CMAKE_CURRENT_SOURCE_DIR}/slam/serialization.h"
)

if(GTSAM_USE_QUATERNIONS)
    set(excluded_sources ${excluded_sources} "${CMAKE_CURRENT_SOURCE_DIR}/geometry/Rot3M.cpp")
else()
    set(excluded_sources ${excluded_sources} "${CMAKE_CURRENT_SOURCE_DIR}/geometry/Rot3Q.cpp")
endif()

# if GTSAM_ENABLE_BOOST_SERIALIZATION is not set, then we need to exclude the following:
if(NOT GTSAM_ENABLE_BOOST_SERIALIZATION)
    list (APPEND excluded_headers
        "${CMAKE_CURRENT_SOURCE_DIR}/gtsam/base/serializationTestHelpers.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/gtsam/base/serialization.h"
    )
endif()
 
# if GTSAM_USE_BOOST_FEATURES is not set, then we need to exclude the following:
if(NOT GTSAM_USE_BOOST_FEATURES)
    list (APPEND excluded_sources
        "${CMAKE_CURRENT_SOURCE_DIR}/inference/graph.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/inference/graph-inl.h"
    )
endif()
 
# Common headers
file(GLOB gtsam_core_headers "*.h")
install(FILES ${gtsam_core_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam)

# assemble core libaries
foreach(subdir ${gtsam_subdirs})
    # Build convenience libraries
    file(GLOB_RECURSE subdir_srcs "${subdir}/*.cpp" "${subdir}/*.h") # Include header files so they show up in Visual Studio
    list(REMOVE_ITEM subdir_srcs ${excluded_sources})
	file(GLOB subdir_test_files "${subdir}/tests/*")
	list(REMOVE_ITEM subdir_srcs ${subdir_test_files}) # Remove test files from sources compiled into library
    gtsam_assign_source_folders("${subdir_srcs}") # Create MSVC structure
    set(${subdir}_srcs ${subdir_srcs})

    # Build local library and tests
    message(STATUS "Building ${subdir}")
    add_subdirectory(${subdir})
endforeach(subdir)

# To add additional sources to gtsam when building the full library (static or shared)
# append the subfolder with _srcs appended to the end to this list
set(gtsam_srcs ${3rdparty_srcs})
foreach(subdir ${gtsam_subdirs})
	list(APPEND gtsam_srcs ${${subdir}_srcs})
endforeach(subdir)
list(APPEND gtsam_srcs ${gtsam_core_headers})

IF(MSVC)
	# Add precompiled header to sources
	include(gtsamAddPch)
	gtsamAddPch("precompiled_header.h" "precompiled_header.cpp" "${gtsam_srcs}")
	list(INSERT gtsam_srcs 0 "precompiled_header.cpp")
ENDIF(MSVC)

# Generate and install config and dllexport files
configure_file(config.h.in config.h)
set(library_name GTSAM) # For substitution in dllexport.h.in
configure_file("${GTSAM_SOURCE_DIR}/cmake/dllexport.h.in" "dllexport.h")
list(APPEND gtsam_srcs "${PROJECT_BINARY_DIR}/config.h" "${PROJECT_BINARY_DIR}/dllexport.h")
install(FILES "${PROJECT_BINARY_DIR}/config.h" "${PROJECT_BINARY_DIR}/dllexport.h" DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam)

if(GTSAM_SUPPORT_NESTED_DISSECTION)
    # target metis-gtsam-if is defined in both cases: embedded metis or system version:
    list(APPEND GTSAM_ADDITIONAL_LIBRARIES metis-gtsam-if)
endif()

# Link to cephes library
list(APPEND GTSAM_ADDITIONAL_LIBRARIES cephes-gtsam-if)

# Versions
set(gtsam_version   ${GTSAM_VERSION_STRING})
set(gtsam_soversion ${GTSAM_VERSION_MAJOR})
message(STATUS "GTSAM Version: ${gtsam_version}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")

# build shared and static versions of the library
message(STATUS "Building GTSAM - as a ${GTSAM_LIBRARY_TYPE} library")

# BUILD_SHARED_LIBS automatically defines static/shared libs
# Here we ensure we control which type of library gtsam is specifically
add_library(gtsam ${GTSAM_LIBRARY_TYPE} ${gtsam_srcs})
target_link_libraries(gtsam PUBLIC ${GTSAM_BOOST_LIBRARIES})
target_link_libraries(gtsam PUBLIC ${GTSAM_ADDITIONAL_LIBRARIES})

# Apply build flags:
gtsam_apply_build_flags(gtsam)

set_target_properties(gtsam PROPERTIES
    OUTPUT_NAME         gtsam
    CLEAN_DIRECT_OUTPUT 1
    VERSION             ${gtsam_version}
    SOVERSION           ${gtsam_soversion}
    LINKER_LANGUAGE     CXX)

# Append Eigen include path to either
# system-eigen, or GTSAM eigen path
target_link_libraries(gtsam PUBLIC Eigen3::Eigen)

# MKL include dir:
if (GTSAM_USE_EIGEN_MKL)
  target_include_directories(gtsam PUBLIC ${MKL_INCLUDE_DIR})
endif()

# Add includes for source directories 'BEFORE' boost and any system include
# paths so that the compiler uses GTSAM headers in our source directory instead
# of any previously installed GTSAM headers.
target_include_directories(gtsam BEFORE PUBLIC
  # main gtsam includes:
  $<BUILD_INTERFACE:${GTSAM_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include/>
  # config.h
  $<BUILD_INTERFACE:${GTSAM_BINARY_DIR}>
  # unit tests:
  $<BUILD_INTERFACE:${GTSAM_SOURCE_DIR}/CppUnitLite>
)
# 3rdparty libraries: use the "system" flag so they are included via "-isystem"
# and warnings (and warnings-considered-errors) in those headers are not
# reported as warnings/errors in our targets:
target_include_directories(gtsam SYSTEM BEFORE PUBLIC
  # SuiteSparse_config
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/SuiteSparse_config>
  $<INSTALL_INTERFACE:include/gtsam/3rdparty/SuiteSparse_config>
  # Spectra
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/Spectra>
  # CCOLAMD
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/CCOLAMD/Include>
  $<INSTALL_INTERFACE:include/gtsam/3rdparty/CCOLAMD>
)

if(WIN32) # Add 'lib' prefix to static library to avoid filename collision with shared library
	if (NOT GTSAM_SHARED_LIB)
		set_target_properties(gtsam PROPERTIES
			PREFIX "lib")
		target_compile_definitions(gtsam PRIVATE GTSAM_IMPORT_STATIC)
	else()
		set_target_properties(gtsam PROPERTIES
			PREFIX ""
			DEFINE_SYMBOL GTSAM_EXPORTS
			RUNTIME_OUTPUT_DIRECTORY "${GTSAM_BINARY_DIR}/bin")
	endif()
endif()

if(WIN32) # library to help with demangling variable names on Windows
  target_link_libraries(gtsam PRIVATE Dbghelp)
endif()

install(
	TARGETS gtsam
	EXPORT GTSAM-exports
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

list(APPEND GTSAM_EXPORTED_TARGETS gtsam)
set(GTSAM_EXPORTED_TARGETS "${GTSAM_EXPORTED_TARGETS}" PARENT_SCOPE)

if(MSVC)
  # For CMake<3.15, when building with MSVC,
  # the compiler warning flags (/W3, /W4) are added by default.
  # This removes the warnings which helps with faster compilation on Windows.
  string(REGEX REPLACE "/W[3|4]" "/w" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
endif()
# Make sure that ccolamd compiles even in face of warnings
# and suppress all warnings from 3rd party code if Release build
if(WIN32)
  set_source_files_properties(${3rdparty_srcs} PROPERTIES COMPILE_FLAGS "/w")
else()
  if("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    # Suppress all warnings from 3rd party sources.
    set_source_files_properties(${3rdparty_srcs} PROPERTIES COMPILE_FLAGS "-w -Wno-everything")
  else()
    set_source_files_properties(${3rdparty_srcs} PROPERTIES COMPILE_FLAGS "-Wno-error")
  endif()
endif()
