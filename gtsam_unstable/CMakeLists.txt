# Build full gtsam_unstable library as a single library
# and also build tests
set (gtsam_unstable_subdirs
    base
    geometry
    linear
    discrete
    dynamics
    nonlinear
    slam
)

if(GTSAM_BUILD_METIS) # Only build partition if metis is built
    set (gtsam_unstable_subdirs ${gtsam_unstable_subdirs} partition)
endif(GTSAM_BUILD_METIS)

set(GTSAM_UNSTABLE_BOOST_LIBRARIES ${GTSAM_BOOST_LIBRARIES})

add_custom_target(check.unstable COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure)

# To exclude a source from the library build (in any subfolder)
# Add the full name to this list, as in the following example
# Sources to remove from builds
set (excluded_sources # "")
    "${CMAKE_CURRENT_SOURCE_DIR}/slam/serialization.cpp"
)

set (excluded_headers # "")
    "${CMAKE_CURRENT_SOURCE_DIR}/slam/serialization.h"
)

# assemble core libraries
foreach(subdir ${gtsam_unstable_subdirs})
    # Build convenience libraries
    file(GLOB subdir_srcs "${subdir}/*.cpp")
	file(GLOB subdir_headers "${subdir}/*.h")
    list(REMOVE_ITEM subdir_srcs ${excluded_sources})
    list(REMOVE_ITEM subdir_headers ${excluded_headers})
    set(${subdir}_srcs ${subdir_srcs} ${subdir_headers})
    gtsam_assign_source_folders("${${subdir}_srcs}") # Create MSVC structure

    # Build local library and tests
    message(STATUS "Building ${subdir}_unstable")
    add_subdirectory(${subdir})
endforeach(subdir)

# assemble gtsam_unstable components
set(gtsam_unstable_srcs
    ${base_srcs}
    ${geometry_srcs}
    ${linear_srcs}
    ${discrete_srcs}
    ${dynamics_srcs}
    ${nonlinear_srcs}
    ${slam_srcs}
)

if(GTSAM_BUILD_METIS) # Only build partition if metis is built
    set (gtsam_unstable_srcs ${gtsam_unstable_srcs} ${partition_srcs})
endif(GTSAM_BUILD_METIS)

# Versions - same as core gtsam library
set(gtsam_unstable_version   ${GTSAM_VERSION_MAJOR}.${GTSAM_VERSION_MINOR}.${GTSAM_VERSION_PATCH})
set(gtsam_unstable_soversion ${GTSAM_VERSION_MAJOR})
message(STATUS "GTSAM_UNSTABLE Version: ${gtsam_unstable_version}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")

# build shared or static versions of the library
if (GTSAM_BUILD_STATIC_LIBRARY)
	message(STATUS "Building GTSAM_UNSTABLE - static")
	add_library(gtsam_unstable STATIC ${gtsam_unstable_srcs})
	set_target_properties(gtsam_unstable PROPERTIES
		OUTPUT_NAME         gtsam_unstable
		CLEAN_DIRECT_OUTPUT 1
		VERSION             ${gtsam_unstable_version}
		SOVERSION           ${gtsam_unstable_soversion})
	if(WIN32) # Add 'lib' prefix to static library to avoid filename collision with shared library
		set_target_properties(gtsam_unstable PROPERTIES
			PREFIX "lib"
			COMPILE_DEFINITIONS GTSAM_UNSTABLE_IMPORT_STATIC)
	endif()
	target_link_libraries(gtsam_unstable gtsam ${GTSAM_UNSTABLE_BOOST_LIBRARIES})
	install(TARGETS gtsam_unstable EXPORT GTSAM-exports ARCHIVE DESTINATION lib)
	list(APPEND GTSAM_EXPORTED_TARGETS gtsam_unstable)
	set(GTSAM_EXPORTED_TARGETS "${GTSAM_EXPORTED_TARGETS}" PARENT_SCOPE)
else()
    message(STATUS "Building GTSAM_UNSTABLE - shared")
    add_library(gtsam_unstable SHARED ${gtsam_unstable_srcs})
    set_target_properties(gtsam_unstable PROPERTIES
        OUTPUT_NAME         gtsam_unstable
        CLEAN_DIRECT_OUTPUT 1
    	VERSION             ${gtsam_unstable_version}
     	SOVERSION           ${gtsam_unstable_soversion})
	if(WIN32)
		set_target_properties(gtsam_unstable PROPERTIES
			PREFIX ""
			DEFINE_SYMBOL GTSAM_UNSTABLE_EXPORTS
			RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
	endif()
    target_link_libraries(gtsam_unstable gtsam ${GTSAM_UNSTABLE_BOOST_LIBRARIES})
    install(TARGETS gtsam_unstable EXPORT GTSAM-exports LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
    list(APPEND GTSAM_EXPORTED_TARGETS gtsam_unstable)
    set(GTSAM_EXPORTED_TARGETS "${GTSAM_EXPORTED_TARGETS}" PARENT_SCOPE)
endif()

# Wrap version for gtsam_unstable
if (GTSAM_INSTALL_MATLAB_TOOLBOX)
    # Set up codegen
    include(GtsamMatlabWrap)

    # Generate, build and install toolbox
    set(mexFlags "${GTSAM_BUILD_MEX_BINARY_FLAGS}")
	if(GTSAM_BUILD_STATIC_LIBRARY)
		list(APPEND mexFlags -DGTSAM_IMPORT_STATIC)
	endif()

    # Wrap
    wrap_and_install_library(gtsam_unstable.h "gtsam" "" "${mexFlags}")
endif(GTSAM_INSTALL_MATLAB_TOOLBOX)

# Build examples
add_subdirectory(examples)

# Build timing
add_subdirectory(timing)
