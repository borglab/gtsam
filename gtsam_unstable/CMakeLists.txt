# Build full gtsam_unstable library as a single library
# and also build tests 
set (gtsam_unstable_subdirs
    base 
    discrete 
    dynamics
    linear
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
    install(TARGETS gtsam_unstable-shared LIBRARY DESTINATION lib )
endif(GTSAM_UNSTABLE_BUILD_SHARED_LIBRARY)

# Wrap version for gtsam_unstable
if (GTSAM_BUILD_WRAP)
    # Set up codegen
    include(GtsamMatlabWrap)
    
    # Wrap codegen
    #usage: wrap mexExecutable mexExtension interfacePath moduleName toolboxPath [mexFlags]
    #  mexExecutable : command to execute mex if on path, use 'mex'
    #  mexExtension  : OS/CPU-dependent extension for MEX binaries
    #  interfacePath : *absolute* path to directory of module interface file
    #  moduleName    : the name of the module, interface file must be called moduleName.h
    #  toolboxPath   : the directory in which to generate the wrappers
    #  [mexFlags]    : extra flags for the mex command
    
    # TODO: generate these includes programmatically
    set(mexFlags "-I${Boost_INCLUDE_DIR} -I${CMAKE_INSTALL_PREFIX}/include -I${CMAKE_INSTALL_PREFIX}/include/gtsam_unstable -I${CMAKE_INSTALL_PREFIX}/include/gtsam_unstable/dynamics -I${CMAKE_INSTALL_PREFIX}/include/gtsam_unstable/discrete -L${CMAKE_INSTALL_PREFIX}/lib -lgtsam -lgtsam_unstable")
    set(toolbox_path ${CMAKE_BINARY_DIR}/wrap/gtsam_unstable)
    set(moduleName gtsam_unstable)
    
    find_mexextension()
    
    # Code generation command
    add_custom_target(wrap_gtsam_unstable ALL COMMAND 
        ${CMAKE_BINARY_DIR}/wrap/wrap 
        ${MEX_COMMAND} 
        ${GTSAM_MEX_BIN_EXTENSION} 
        ${CMAKE_CURRENT_SOURCE_DIR} 
        ${moduleName} 
        ${toolbox_path} 
        "${mexFlags}"
        DEPENDS wrap)
    
    # Build command
    if (GTSAM_ENABLE_BUILD_MEX_BINARIES)
        # Actually compile the mex files when building the library
        if (GTSAM_ENABLE_BUILD_MEX_BINARIES_ALL)    
            add_custom_target(wrap_gtsam_unstable_build ALL
                COMMAND make ${GTSAM_BUILD_MEX_BINARY_FLAGS} 
                WORKING_DIRECTORY ${toolbox_path} 
                DEPENDS wrap_gtsam_unstable)
        else()            
            add_custom_target(wrap_gtsam_unstable_build
                COMMAND make ${GTSAM_BUILD_MEX_BINARY_FLAGS} 
                WORKING_DIRECTORY ${toolbox_path} 
                DEPENDS wrap_gtsam_unstable)
        endif()
    endif (GTSAM_ENABLE_BUILD_MEX_BINARIES)    
    
    if (GTSAM_INSTALL_MATLAB_TOOLBOX)
        # Primary toolbox files
        message(STATUS "Installing Matlab Toolbox to ${GTSAM_TOOLBOX_INSTALL_PATH}")
        install(DIRECTORY DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH}) # make an empty folder
        # exploit need for trailing slash to specify a full folder, rather than just its contents to copy
        install(DIRECTORY ${toolbox_path} DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH})
    endif (GTSAM_INSTALL_MATLAB_TOOLBOX)
endif(GTSAM_BUILD_WRAP)
