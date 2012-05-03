
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
    ${CMAKE_BINARY_DIR}/wrap/wrap ${GTSAM_MEX_BIN_EXTENSION} ${CMAKE_CURRENT_SOURCE_DIR} ${moduleName} ${toolbox_path} "${mexFlags}"
    DEPENDS wrap)

if (GTSAM_INSTALL_MATLAB_TOOLBOX)
    # Primary toolbox files
    message(STATUS "Installing Matlab Toolbox to ${GTSAM_TOOLBOX_INSTALL_PATH}")
    install(DIRECTORY DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH}) # make an empty folder
    # exploit need for trailing slash to specify a full folder, rather than just its contents to copy
    install(DIRECTORY ${toolbox_path} DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH})
endif (GTSAM_INSTALL_MATLAB_TOOLBOX)
