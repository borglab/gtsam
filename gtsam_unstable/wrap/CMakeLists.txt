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
set(mexFlags "-I${Boost_INCLUDE_DIR} -I${Wrap_INCLUDE_DIR} -I${CMAKE_INSTALL_PREFIX}/include/gtsam_unstable -I${CMAKE_INSTALL_PREFIX}/include/gtsam_unstable/dynamics -I${CMAKE_INSTALL_PREFIX}/include/gtsam_unstable/discrete -L${CMAKE_INSTALL_PREFIX}/lib -lgtsam -lgtsam_unstable")
set(toolbox_path ${CMAKE_BINARY_DIR}/wrap/gtsam_unstable)
set(moduleName gtsam_unstable)

find_mexextension()

# Code generation command
add_custom_target(wrap_gtsam_unstable ALL COMMAND 
    ${Wrap_CMD} ${GTSAM_MEX_BIN_EXTENSION} ${CMAKE_CURRENT_SOURCE_DIR}/../ ${moduleName} ${toolbox_path} "${mexFlags}")

option(gtsam_unstable_INSTALL_MATLAB_TOOLBOX  "Enable/Disable installation of matlab toolbox"  ON)
option(gtsam_unstable_INSTALL_MATLAB_EXAMPLES "Enable/Disable installation of matlab examples" ON)
option(gtsam_unstable_INSTALL_MATLAB_TESTS    "Enable/Disable installation of matlab tests"    ON)

set(gtsam_unstable_TOOLBOX_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/borg/toolbox CACHE DOCSTRING "Path to install matlab toolbox")

if (gtsam_unstable_INSTALL_MATLAB_TOOLBOX)
    # Primary toolbox files
    message(STATUS "Installing Matlab Toolbox to ${gtsam_unstable_TOOLBOX_INSTALL_PATH}")
    install(DIRECTORY DESTINATION ${gtsam_unstable_TOOLBOX_INSTALL_PATH}) # make an empty folder
    # exploit need for trailing slash to specify a full folder, rather than just its contents to copy
    install(DIRECTORY ${toolbox_path} DESTINATION ${gtsam_unstable_TOOLBOX_INSTALL_PATH})
    
    # Examples
    if (gtsam_unstable_INSTALL_MATLAB_EXAMPLES)
        message(STATUS "Installing Matlab Toolbox Examples")
        file(GLOB matlab_examples "${CMAKE_SOURCE_DIR}/examples/matlab/*.m")
        install(FILES ${matlab_examples} DESTINATION ${gtsam_unstable_TOOLBOX_INSTALL_PATH}/gtsam_unstable/examples)
    endif (gtsam_unstable_INSTALL_MATLAB_EXAMPLES)
    
    # Tests
    if (gtsam_unstable_INSTALL_MATLAB_TESTS)
        message(STATUS "Installing Matlab Toolbox Tests")
        file(GLOB matlab_tests "${CMAKE_SOURCE_DIR}/tests/matlab/*.m")
        install(FILES ${matlab_tests} DESTINATION ${gtsam_unstable_TOOLBOX_INSTALL_PATH}/gtsam_unstable/tests)
    endif (gtsam_unstable_INSTALL_MATLAB_TESTS)
endif (gtsam_unstable_INSTALL_MATLAB_TOOLBOX)
