# Install matlab components

# Tests
message(STATUS "Installing Matlab Toolbox Tests")
install(DIRECTORY "${GTSAM_SOURCE_ROOT_DIR}/matlab/gtsam_tests" DESTINATION "${GTSAM_TOOLBOX_INSTALL_PATH}" FILES_MATCHING PATTERN "*.m" PATTERN ".svn" EXCLUDE)

# Examples
message(STATUS "Installing Matlab Toolbox Examples")
# Matlab files: *.m and *.fig
install(DIRECTORY "${GTSAM_SOURCE_ROOT_DIR}/matlab/gtsam_examples" DESTINATION "${GTSAM_TOOLBOX_INSTALL_PATH}" FILES_MATCHING PATTERN "*.m" PATTERN "*.fig" PATTERN ".svn" EXCLUDE)

# Utilities
message(STATUS "Installing Matlab Toolbox Utilities")
install(DIRECTORY "${GTSAM_SOURCE_ROOT_DIR}/matlab/+gtsam" DESTINATION "${GTSAM_TOOLBOX_INSTALL_PATH}" FILES_MATCHING PATTERN "*.m" PATTERN ".svn" EXCLUDE)
install(FILES "${GTSAM_SOURCE_ROOT_DIR}/matlab/README-gtsam-toolbox.txt" DESTINATION "${GTSAM_TOOLBOX_INSTALL_PATH}")

message(STATUS "Installing Matlab Toolbox Examples (Data)")
# Data files: *.graph and *.txt
file(GLOB matlab_examples_data_graph "${GTSAM_SOURCE_ROOT_DIR}/examples/Data/*.graph")
file(GLOB matlab_examples_data_txt "${GTSAM_SOURCE_ROOT_DIR}/examples/Data/*.txt")
set(matlab_examples_data ${matlab_examples_data_graph} ${matlab_examples_data_txt}) 
install(FILES ${matlab_examples_data} DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH}/gtsam_examples/Data)
