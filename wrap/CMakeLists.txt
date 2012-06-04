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
#usage: wrap mexExtension interfacePath moduleName toolboxPath
#  mexExtension  : OS/CPU-dependent extension for MEX binaries
#  interfacePath : *absolute* path to directory of module interface file
#  moduleName    : the name of the module, interface file must be called moduleName.h
#  toolboxPath   : the directory in which to generate the wrappers
#  [mexFlags]    : extra flags for the mex command

set(mexFlags "-I${Boost_INCLUDE_DIR} -I${CMAKE_INSTALL_PREFIX}/include -I${CMAKE_INSTALL_PREFIX}/include/gtsam -I${CMAKE_INSTALL_PREFIX}/include/gtsam/base -I${CMAKE_INSTALL_PREFIX}/include/gtsam/geometry -I${CMAKE_INSTALL_PREFIX}/include/gtsam/linear -I${CMAKE_INSTALL_PREFIX}/include/gtsam/nonlinear -I${CMAKE_INSTALL_PREFIX}/include/gtsam/slam -L${CMAKE_INSTALL_PREFIX}/lib -lgtsam")
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
    ${WRAP_EXE} ${GTSAM_MEX_BIN_EXTENSION} ${CMAKE_CURRENT_SOURCE_DIR}/../ ${moduleName} ${toolbox_path} "${mexFlags}"
    DEPENDS wrap)

# Build command
# Experimental: requires matlab to be on your path
if (GTSAM_ENABLE_BUILD_MEX_BINARIES)
    # Actually compile the mex files when building the library
    set(TOOLBOX_MAKE_FLAGS "-j2")    
    add_custom_target(wrap_gtsam_build 
        COMMAND make ${TOOLBOX_MAKE_FLAGS} 
        WORKING_DIRECTORY ${toolbox_path} 
        DEPENDS wrap_gtsam)
endif (GTSAM_ENABLE_BUILD_MEX_BINARIES)    

set(GTSAM_TOOLBOX_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/borg/toolbox CACHE DOCSTRING "Path to install matlab toolbox")

if (GTSAM_INSTALL_MATLAB_TOOLBOX)
    # Primary toolbox files
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
        set(data_excludes "${CMAKE_SOURCE_DIR}/examples/Data/.svn")
        file(GLOB matlab_examples_data "${CMAKE_SOURCE_DIR}/examples/Data/*.*")
        list(REMOVE_ITEM matlab_examples_data ${data_excludes})
        install(FILES ${matlab_examples_data} DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH}/gtsam/examples/Data)
    endif (GTSAM_INSTALL_MATLAB_EXAMPLES)
    
    # Tests
    if (GTSAM_INSTALL_MATLAB_TESTS)
        message(STATUS "Installing Matlab Toolbox Tests")
        file(GLOB matlab_tests "${CMAKE_SOURCE_DIR}/tests/matlab/*.m")
        install(FILES ${matlab_tests} DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH}/gtsam/tests)
    endif (GTSAM_INSTALL_MATLAB_TESTS)
endif (GTSAM_INSTALL_MATLAB_TOOLBOX)
