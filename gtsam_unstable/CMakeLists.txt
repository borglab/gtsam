# Build full gtsam_unstable library as a single library
# and also build tests 
set (gtsam_unstable_subdirs
    base 
    discrete 
    dynamics
    linear
    nonlinear
    slam 
)

add_custom_target(check.unstable COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure)

# assemble core libaries
foreach(subdir ${gtsam_unstable_subdirs})
    # Build convenience libraries
    file(GLOB subdir_srcs "${subdir}/*.cpp")
    set(${subdir}_srcs ${subdir_srcs})
    if (GTSAM_BUILD_CONVENIENCE_LIBRARIES)
        message(STATUS "Building Convenience Library: ${subdir}_unstable")
        add_library("${subdir}_unstable" STATIC ${subdir_srcs})
    endif()

    # Build local library and tests
    message(STATUS "Building ${subdir}_unstable") 
    add_subdirectory(${subdir})
endforeach(subdir)
 
# assemble gtsam_unstable components
set(gtsam_unstable_srcs
    ${base_srcs} 
    ${discrete_srcs}
    ${dynamics_srcs}
    ${linear_srcs}    
    ${slam_srcs} 
)

option (GTSAM_UNSTABLE_BUILD_SHARED_LIBRARY "Enable/Disable building of a shared version of gtsam_unstable" ON)

# Versions - same as core gtsam library
set(gtsam_unstable_version   ${GTSAM_VERSION_MAJOR}.${GTSAM_VERSION_MINOR}.${GTSAM_VERSION_PATCH})
set(gtsam_unstable_soversion ${GTSAM_VERSION_MAJOR})
message(STATUS "GTSAM_UNSTABLE Version: ${gtsam_unstable_version}")
message(STATUS "Install prefix: ${CMAKE_INSTALL_PREFIX}")
 
# build shared and static versions of the library
message(STATUS "Building GTSAM_UNSTABLE - static")
add_library(gtsam_unstable-static STATIC ${gtsam_unstable_srcs})
set_target_properties(gtsam_unstable-static PROPERTIES 
    OUTPUT_NAME         gtsam_unstable
    CLEAN_DIRECT_OUTPUT 1
	VERSION             ${gtsam_unstable_version}
 	SOVERSION           ${gtsam_unstable_soversion})
target_link_libraries(gtsam_unstable-static gtsam-static)
install(TARGETS gtsam_unstable-static ARCHIVE DESTINATION lib)

if (GTSAM_UNSTABLE_BUILD_SHARED_LIBRARY)
    message(STATUS "Building GTSAM_UNSTABLE - shared")
    add_library(gtsam_unstable-shared SHARED ${gtsam_unstable_srcs})
    set_target_properties(gtsam_unstable-shared PROPERTIES 
        OUTPUT_NAME         gtsam_unstable
        CLEAN_DIRECT_OUTPUT 1
    	VERSION             ${gtsam_unstable_version}
     	SOVERSION           ${gtsam_unstable_soversion})
    target_link_libraries(gtsam_unstable-shared gtsam-shared)
    install(TARGETS gtsam_unstable-shared LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
endif(GTSAM_UNSTABLE_BUILD_SHARED_LIBRARY)

# Wrap version for gtsam_unstable
if (GTSAM_BUILD_WRAP)
    # Set up codegen
    include(GtsamMatlabWrap)
    
    # TODO: generate these includes programmatically
    # Choose include flags depending on build process
    if (GTSAM_BUILD_MEX_BIN)
        set(MEX_INCLUDE_ROOT ${GTSAM_SOURCE_ROOT_DIR})
        set(MEX_LIB_ROOT ${CMAKE_BINARY_DIR})
        set(GTSAM_LIB_DIR ${MEX_LIB_ROOT}/gtsam)
        set(GTSAM_UNSTABLE_LIB_DIR ${MEX_LIB_ROOT}/gtsam_unstable)
    else()
        set(MEX_INCLUDE_ROOT ${CMAKE_INSTALL_PREFIX}/include)
        set(MEX_LIB_ROOT ${CMAKE_INSTALL_PREFIX}/lib)
        set(GTSAM_LIB_DIR ${MEX_LIB_ROOT})
        set(GTSAM_UNSTABLE_LIB_DIR ${MEX_LIB_ROOT})
    endif()
    
    # Generate, build and install toolbox
    set(mexFlags "-I${Boost_INCLUDE_DIR} -I${MEX_INCLUDE_ROOT} -I${MEX_INCLUDE_ROOT}/gtsam_unstable -I${MEX_INCLUDE_ROOT}/gtsam_unstable/dynamics -I${MEX_INCLUDE_ROOT}/gtsam_unstable/discrete -L${GTSAM_UNSTABLE_LIB_DIR} -L${GTSAM_LIB_DIR} -lgtsam -lgtsam_unstable")
    
    # Macro to handle details of setting up targets
    wrap_library(gtsam_unstable "${mexFlags}" "./")

endif(GTSAM_BUILD_WRAP)
