# Build/install Wrap

# Build the executable itself
file(GLOB wrap_srcs "*.cpp")
list(REMOVE_ITEM wrap_srcs wrap.cpp)
add_library(wrap_lib STATIC ${wrap_srcs})
add_executable(wrap wrap.cpp)
target_link_libraries(wrap wrap_lib ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY})

# Install wrap binary
if (GTSAM_INSTALL_WRAP)
    install(TARGETS wrap DESTINATION bin)
endif(GTSAM_INSTALL_WRAP)

# Install matlab header
install(FILES matlab.h DESTINATION include/wrap)

# Build tests
if (GTSAM_BUILD_TESTS) 
    add_definitions(-DTOPSRCDIR="${CMAKE_SOURCE_DIR}")
    set(wrap_local_libs wrap_lib) 
    gtsam_add_subdir_tests("wrap" "${wrap_local_libs}" "${wrap_local_libs}" "") 
endif(GTSAM_BUILD_TESTS)

# Wrap codegen
#usage: wrap mexExecutable mexExtension interfacePath moduleName toolboxPath [mexFlags]
#  mexExecutable : command to execute mex if on path, use 'mex'
#  mexExtension  : OS/CPU-dependent extension for MEX binaries
#  interfacePath : *absolute* path to directory of module interface file
#  moduleName    : the name of the module, interface file must be called moduleName.h
#  toolboxPath   : the directory in which to generate the wrappers
#  [mexFlags]    : extra flags for the mex command

# Choose include flags depending on build process
if (GTSAM_BUILD_MEX_BIN)
    set(MEX_INCLUDE_ROOT ${GTSAM_SOURCE_ROOT_DIR})
    set(MEX_LIB_ROOT ${CMAKE_BINARY_DIR})
else()
    set(MEX_INCLUDE_ROOT ${CMAKE_INSTALL_PREFIX}/include)
    set(MEX_LIB_ROOT ${CMAKE_INSTALL_PREFIX}/lib)
endif()

set(mexFlags "-I${Boost_INCLUDE_DIR} -I${MEX_INCLUDE_ROOT} -I${MEX_INCLUDE_ROOT}/gtsam -I${MEX_INCLUDE_ROOT}/gtsam/base -I${MEX_INCLUDE_ROOT}/gtsam/geometry -I${MEX_INCLUDE_ROOT}/gtsam/linear -I${MEX_INCLUDE_ROOT}/gtsam/discrete -I${MEX_INCLUDE_ROOT}/gtsam/inference -I${MEX_INCLUDE_ROOT}/gtsam/nonlinear -I${MEX_INCLUDE_ROOT}/gtsam/slam -L${MEX_LIB_ROOT}/gtsam -lgtsam")
if(MSVC OR CYGWIN OR WINGW)
  set(mexFlags "${mexFlags} LINKFLAGS='$LINKFLAGS /LIBPATH:${Boost_LIBRARY_DIRS}'")
endif()
set(toolbox_path ${CMAKE_BINARY_DIR}/wrap/gtsam)
set(moduleName gtsam)

include(GtsamMatlabWrap)
find_mexextension()

# get binary path for wrap executable to allow for different build configurations (e.g., ROS)
if (NOT EXECUTABLE_OUTPUT_PATH)
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR})
endif()

# Code generation command
get_property(WRAP_EXE TARGET wrap PROPERTY LOCATION)
add_custom_target(wrap_gtsam ALL COMMAND 
    ${WRAP_EXE}
    ${MEX_COMMAND}
    ${GTSAM_MEX_BIN_EXTENSION} 
    ${CMAKE_CURRENT_SOURCE_DIR}/../ 
    ${moduleName} 
    ${toolbox_path} 
    "${mexFlags}"
    DEPENDS wrap)

# Build command
if (GTSAM_BUILD_MEX_BIN)
    # Actually compile the mex files when building the library
    # TODO: pass correct make flags from parent process
    message(STATUS "Building Matlab MEX binaries for toolbox with flags ${GTSAM_BUILD_MEX_BINARY_FLAGS}")
    if (GTSAM_INSTALL_MEX_BIN)    
        add_custom_target(wrap_gtsam_build ALL
            COMMAND make ${GTSAM_BUILD_MEX_BINARY_FLAGS} 
            WORKING_DIRECTORY ${toolbox_path} 
            DEPENDS wrap_gtsam)    
    else()
        add_custom_target(wrap_gtsam_build
            COMMAND make ${GTSAM_BUILD_MEX_BINARY_FLAGS} 
            WORKING_DIRECTORY ${toolbox_path} 
            DEPENDS wrap_gtsam)
    endif()
endif ()    

if (GTSAM_INSTALL_MATLAB_TOOLBOX)
    # Primary toolbox files
    # Note that we copy the entire contents of the folder blindly - this is because 
    # the generated files won't exist at configuration time
    message(STATUS "Installing Matlab Toolbox to ${GTSAM_TOOLBOX_INSTALL_PATH}")
    install(DIRECTORY DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH}) # make an empty folder
    # exploit need for trailing slash to specify a full folder, rather than just its contents to copy
    install(DIRECTORY ${toolbox_path} DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH})
    
    # Examples
    if (GTSAM_INSTALL_MATLAB_EXAMPLES)
        message(STATUS "Installing Matlab Toolbox Examples")
        file(GLOB matlab_examples "${CMAKE_SOURCE_DIR}/examples/matlab/*.m")
        install(FILES ${matlab_examples} DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH}/gtsam/examples)
        
        message(STATUS "Installing Matlab Toolbox Examples (Data)")
        # Data files: *.graph and *.txt
        file(GLOB matlab_examples_data_graph "${CMAKE_SOURCE_DIR}/examples/Data/*.graph")
        file(GLOB matlab_examples_data_txt "${CMAKE_SOURCE_DIR}/examples/Data/*.txt")
        set(matlab_examples_data ${matlab_examples_data_graph} ${matlab_examples_data_txt}) 
        install(FILES ${matlab_examples_data} DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH}/gtsam/Data)
    endif (GTSAM_INSTALL_MATLAB_EXAMPLES)
    
    # Tests
    if (GTSAM_INSTALL_MATLAB_TESTS)
        message(STATUS "Installing Matlab Toolbox Tests")
        file(GLOB matlab_tests "${CMAKE_SOURCE_DIR}/tests/matlab/*.m")
        install(FILES ${matlab_tests} DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH}/gtsam/tests)
    endif (GTSAM_INSTALL_MATLAB_TESTS)
endif (GTSAM_INSTALL_MATLAB_TOOLBOX)
