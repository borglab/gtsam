# Build full gtsam_unstable library as a single library
# and also build tests 
set (gtsam_unstable_subdirs
    base 
    geometry
    discrete
    linear 
    dynamics
    nonlinear
    slam
    navigation
)

set(GTSAM_UNSTABLE_BOOST_LIBRARIES ${GTSAM_BOOST_LIBRARIES} ${Boost_THREAD_LIBRARY})

add_custom_target(check.unstable COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure)

# assemble core libaries
foreach(subdir ${gtsam_unstable_subdirs})
    # Build convenience libraries
    file(GLOB subdir_srcs "${subdir}/*.cpp")
	file(GLOB subdir_headers "${subdir}/*.h")
    set(${subdir}_srcs ${subdir_srcs} ${subdir_headers})
    gtsam_assign_source_folders("${${subdir}_srcs}") # Create MSVC structure
    if (subdir_srcs AND GTSAM_BUILD_CONVENIENCE_LIBRARIES)
        message(STATUS "Building Convenience Library: ${subdir}_unstable")
        add_library("${subdir}_unstable" STATIC ${${subdir}_srcs})
    endif()

    # Build local library and tests
    message(STATUS "Building ${subdir}_unstable") 
    add_subdirectory(${subdir})
endforeach(subdir)
 
# assemble gtsam_unstable components
set(gtsam_unstable_srcs
    ${base_srcs} 
    ${geometry_srcs}
    ${discrete_srcs}
    ${dynamics_srcs}
    ${linear_srcs}    
    ${nonlinear_srcs}
    ${slam_srcs}
    ${navigation_srcs}
)

# Versions - same as core gtsam library
set(gtsam_unstable_version   ${GTSAM_VERSION_MAJOR}.${GTSAM_VERSION_MINOR}.${GTSAM_VERSION_PATCH})
set(gtsam_unstable_soversion ${GTSAM_VERSION_MAJOR})
message(STATUS "GTSAM_UNSTABLE Version: ${gtsam_unstable_version}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")
 
# build shared and static versions of the library
if (GTSAM_BUILD_STATIC_LIBRARY)
	message(STATUS "Building GTSAM_UNSTABLE - static")
	add_library(gtsam_unstable-static STATIC ${gtsam_unstable_srcs})
	set_target_properties(gtsam_unstable-static PROPERTIES 
		OUTPUT_NAME         gtsam_unstable
		CLEAN_DIRECT_OUTPUT 1
		VERSION             ${gtsam_unstable_version}
		SOVERSION           ${gtsam_unstable_soversion})
	if(WIN32) # Add 'lib' prefix to static library to avoid filename collision with shared library
		set_target_properties(gtsam_unstable-static PROPERTIES PREFIX "lib")
	endif()
	target_link_libraries(gtsam_unstable-static gtsam-static ${GTSAM_UNSTABLE_BOOST_LIBRARIES})
	install(TARGETS gtsam_unstable-static EXPORT GTSAM-exports ARCHIVE DESTINATION lib)
	list(APPEND GTSAM_EXPORTED_TARGETS gtsam_unstable-static)
	set(GTSAM_EXPORTED_TARGETS "${GTSAM_EXPORTED_TARGETS}" PARENT_SCOPE)
endif()

if (GTSAM_BUILD_SHARED_LIBRARY)
    message(STATUS "Building GTSAM_UNSTABLE - shared")
    add_library(gtsam_unstable-shared SHARED ${gtsam_unstable_srcs})
    set_target_properties(gtsam_unstable-shared PROPERTIES 
        OUTPUT_NAME         gtsam_unstable
        CLEAN_DIRECT_OUTPUT 1
    	VERSION             ${gtsam_unstable_version}
     	SOVERSION           ${gtsam_unstable_soversion})
	if(WIN32)
		set_target_properties(gtsam_unstable-shared PROPERTIES
			PREFIX ""
			DEFINE_SYMBOL GTSAM_UNSTABLE_EXPORTS
			RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin")
		add_definitions(/wd4251 /wd4275) # Disable non-DLL-exported base class warnings
	endif()
    target_link_libraries(gtsam_unstable-shared gtsam-shared ${GTSAM_UNSTABLE_BOOST_LIBRARIES})
    install(TARGETS gtsam_unstable-shared EXPORT GTSAM-exports LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
    list(APPEND GTSAM_EXPORTED_TARGETS gtsam_unstable-shared)
    set(GTSAM_EXPORTED_TARGETS "${GTSAM_EXPORTED_TARGETS}" PARENT_SCOPE)
endif()

# Wrap version for gtsam_unstable
if (GTSAM_INSTALL_MATLAB_TOOLBOX)
    # Set up codegen
    include(GtsamMatlabWrap)
    
    # TODO: generate these includes programmatically
    # Choose include flags depending on build process
	set(MEX_INCLUDE_ROOT ${GTSAM_SOURCE_ROOT_DIR})
	set(MEX_LIB_ROOT ${CMAKE_BINARY_DIR})    # FIXME: is this used?
	set(GTSAM_LIB_DIR ${MEX_LIB_ROOT}/gtsam) # FIXME: is this used?
	set(GTSAM_UNSTABLE_LIB_DIR ${MEX_LIB_ROOT}/gtsam_unstable) # FIXME: is this used?
    
    # Generate, build and install toolbox
    set(mexFlags -I${MEX_INCLUDE_ROOT} -I${Boost_INCLUDE_DIR})
    
    # Macro to handle details of setting up targets
    wrap_library(gtsam_unstable "${mexFlags}" "./" gtsam)

endif(GTSAM_INSTALL_MATLAB_TOOLBOX)
