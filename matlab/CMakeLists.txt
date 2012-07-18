# Install matlab components

# Utility functions
message(STATUS "Installing Matlab Utility Functions")
# Matlab files: *.m and *.fig
file(GLOB matlab_utils_m "${GTSAM_SOURCE_ROOT_DIR}/matlab/*.m")
file(GLOB matlab_utils_fig "${GTSAM_SOURCE_ROOT_DIR}/matlab/*.fig")
set(matlab_utils ${matlab_utils_m} ${matlab_utils_fig})
install(FILES ${matlab_utils} DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH}/)

# Tests
message(STATUS "Installing Matlab Toolbox Tests")
file(GLOB matlab_tests "${GTSAM_SOURCE_ROOT_DIR}/matlab/tests/*.m")
install(FILES ${matlab_tests} DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH}/gtsam_tests)

# Examples
message(STATUS "Installing Matlab Toolbox Examples")
# Matlab files: *.m and *.fig
file(GLOB matlab_examples_m "${GTSAM_SOURCE_ROOT_DIR}/matlab/examples/*.m")
file(GLOB matlab_examples_fig "${GTSAM_SOURCE_ROOT_DIR}/matlab/examples/*.fig")
set(matlab_examples ${matlab_examples_m} ${matlab_examples_fig})
install(FILES ${matlab_examples} DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH}/gtsam_examples)

message(STATUS "Installing Matlab Toolbox Examples (Data)")
# Data files: *.graph and *.txt
file(GLOB matlab_examples_data_graph "${GTSAM_SOURCE_ROOT_DIR}/examples/Data/*.graph")
file(GLOB matlab_examples_data_txt "${GTSAM_SOURCE_ROOT_DIR}/examples/Data/*.txt")
set(matlab_examples_data ${matlab_examples_data_graph} ${matlab_examples_data_txt}) 
install(FILES ${matlab_examples_data} DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH}/gtsam_examples/Data)
