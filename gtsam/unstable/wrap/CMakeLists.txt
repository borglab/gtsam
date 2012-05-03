# Find wrap
find_package(Wrap REQUIRED)

# Set up codegen
include(GtsamMatlabWrap)

# Wrap codegen
#usage: wrap mexExtension interfacePath moduleName toolboxPath
#  mexExtension  : OS/CPU-dependent extension for MEX binaries
#  interfacePath : *absolute* path to directory of module interface file
#  moduleName    : the name of the module, interface file must be called moduleName.h
#  toolboxPath   : the directory in which to generate the wrappers
#  [mexFlags]    : extra flags for the mex command

# TODO: generate these includes programmatically
set(mexFlags "-I${Boost_INCLUDE_DIR} -I${Wrap_INCLUDE_DIR} -I${CMAKE_INSTALL_PREFIX}/include/gtsam2 -I${CMAKE_INSTALL_PREFIX}/include/gtsam2/dynamics -I${CMAKE_INSTALL_PREFIX}/include/gtsam2/discrete -L${CMAKE_INSTALL_PREFIX}/lib -lgtsam -lgtsam2")
set(toolbox_path ${CMAKE_BINARY_DIR}/wrap/gtsam2)
set(moduleName gtsam2)

find_mexextension()

# Code generation command
add_custom_target(wrap_gtsam2 ALL COMMAND 
    ${Wrap_CMD} ${GTSAM_MEX_BIN_EXTENSION} ${CMAKE_CURRENT_SOURCE_DIR}/../ ${moduleName} ${toolbox_path} "${mexFlags}")

option(GTSAM2_INSTALL_MATLAB_TOOLBOX  "Enable/Disable installation of matlab toolbox"  ON)
option(GTSAM2_INSTALL_MATLAB_EXAMPLES "Enable/Disable installation of matlab examples" ON)
option(GTSAM2_INSTALL_MATLAB_TESTS    "Enable/Disable installation of matlab tests"    ON)

set(GTSAM2_TOOLBOX_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/borg/toolbox CACHE DOCSTRING "Path to install matlab toolbox")

if (GTSAM2_INSTALL_MATLAB_TOOLBOX)
    # Primary toolbox files
    message(STATUS "Installing Matlab Toolbox to ${GTSAM2_TOOLBOX_INSTALL_PATH}")
    install(DIRECTORY DESTINATION ${GTSAM2_TOOLBOX_INSTALL_PATH}) # make an empty folder
    # exploit need for trailing slash to specify a full folder, rather than just its contents to copy
    install(DIRECTORY ${toolbox_path} DESTINATION ${GTSAM2_TOOLBOX_INSTALL_PATH})
    
    # Examples
    if (GTSAM2_INSTALL_MATLAB_EXAMPLES)
        message(STATUS "Installing Matlab Toolbox Examples")
        file(GLOB matlab_examples "${CMAKE_SOURCE_DIR}/examples/matlab/*.m")
        install(FILES ${matlab_examples} DESTINATION ${GTSAM2_TOOLBOX_INSTALL_PATH}/gtsam2/examples)
    endif (GTSAM2_INSTALL_MATLAB_EXAMPLES)
    
    # Tests
    if (GTSAM2_INSTALL_MATLAB_TESTS)
        message(STATUS "Installing Matlab Toolbox Tests")
        file(GLOB matlab_tests "${CMAKE_SOURCE_DIR}/tests/matlab/*.m")
        install(FILES ${matlab_tests} DESTINATION ${GTSAM2_TOOLBOX_INSTALL_PATH}/gtsam2/tests)
    endif (GTSAM2_INSTALL_MATLAB_TESTS)
endif (GTSAM2_INSTALL_MATLAB_TOOLBOX)
