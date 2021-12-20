project(gtsam LANGUAGES CXX)

# We split the library in to separate subfolders, each containing
# tests, timing, and an optional convenience library.
# The following variable is the master list of subdirs to add
set (gtsam_subdirs
    base
    geometry
    inference
    symbolic
    discrete
    linear
    nonlinear
    sam
    sfm
    slam
    smart
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
    list(APPEND GTSAM_ADDITIONAL_LIBRARIES metis-gtsam)
endif()

# Versions
set(gtsam_version   ${GTSAM_VERSION_MAJOR}.${GTSAM_VERSION_MINOR}.${GTSAM_VERSION_PATCH})
set(gtsam_soversion ${GTSAM_VERSION_MAJOR})
message(STATUS "GTSAM Version: ${gtsam_version}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")

# build shared and static versions of the library
message(STATUS "Building GTSAM - shared: ${BUILD_SHARED_LIBS}")

# BUILD_SHARED_LIBS automatically defines static/shared libs:
add_library(gtsam ${gtsam_srcs})
target_link_libraries(gtsam PUBLIC ${GTSAM_BOOST_LIBRARIES})
target_link_libraries(gtsam PUBLIC ${GTSAM_ADDITIONAL_LIBRARIES})

# Apply build flags:
gtsam_apply_build_flags(gtsam)

set_target_properties(gtsam PROPERTIES
    OUTPUT_NAME         gtsam
    CLEAN_DIRECT_OUTPUT 1
    VERSION             ${gtsam_version}
    SOVERSION           ${gtsam_soversion})

# Append Eigen include path, set in top-level CMakeLists.txt to either
# system-eigen, or GTSAM eigen path
target_include_directories(gtsam PUBLIC
  $<BUILD_INTERFACE:${GTSAM_EIGEN_INCLUDE_FOR_BUILD}>
  $<INSTALL_INTERFACE:${GTSAM_EIGEN_INCLUDE_FOR_INSTALL}>
)
# MKL include dir:
if (GTSAM_USE_EIGEN_MKL)
  target_include_directories(gtsam PUBLIC ${MKL_INCLUDE_DIR})
endif()

if(GTSAM_USE_TBB)
  target_include_directories(gtsam PUBLIC ${TBB_INCLUDE_DIRS})
endif()

# Add includes for source directories 'BEFORE' boost and any system include
# paths so that the compiler uses GTSAM headers in our source directory instead
# of any previously installed GTSAM headers.
target_include_directories(gtsam BEFORE PUBLIC
  # SuiteSparse_config
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/SuiteSparse_config>
  $<INSTALL_INTERFACE:include/gtsam/3rdparty/SuiteSparse_config>
  # CCOLAMD
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/CCOLAMD/Include>
  $<INSTALL_INTERFACE:include/gtsam/3rdparty/CCOLAMD>
  # main gtsam includes:
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include/>
  # config.h
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}>
  # unit tests:
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/CppUnitLite>
)
if(GTSAM_SUPPORT_NESTED_DISSECTION)
  target_include_directories(gtsam BEFORE PUBLIC
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/gtsam/3rdparty/metis/include>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/gtsam/3rdparty/metis/libmetis>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/gtsam/3rdparty/metis/GKlib>
    $<INSTALL_INTERFACE:include/gtsam/3rdparty/metis/>
  )
endif()



if(WIN32) # Add 'lib' prefix to static library to avoid filename collision with shared library
	if (NOT BUILD_SHARED_LIBS)
		set_target_properties(gtsam PROPERTIES
			PREFIX "lib"
			COMPILE_DEFINITIONS GTSAM_IMPORT_STATIC)
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

# Make sure that ccolamd compiles even in face of warnings
# and suppress all warnings from 3rd party code if Release build
if(WIN32)
  set_source_files_properties(${3rdparty_srcs} PROPERTIES COMPILE_FLAGS "/w")
else()
  if("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    # Suppress all warnings from 3rd party sources.
    set_source_files_properties(${3rdparty_srcs} PROPERTIES COMPILE_FLAGS "-w")
  else()
    set_source_files_properties(${3rdparty_srcs} PROPERTIES COMPILE_FLAGS "-Wno-error")
  endif()
endif()

# Create the matlab toolbox for the gtsam library
if (GTSAM_INSTALL_MATLAB_TOOLBOX)
    # Set up codegen
    include(GtsamMatlabWrap)

    # Generate, build and install toolbox
    set(mexFlags "${GTSAM_BUILD_MEX_BINARY_FLAGS}")
        if(NOT BUILD_SHARED_LIBS)
                list(APPEND mexFlags -DGTSAM_IMPORT_STATIC)
        endif()

    # Wrap
    wrap_and_install_library(../gtsam.h "${GTSAM_ADDITIONAL_LIBRARIES}" "" "${mexFlags}")
endif ()
