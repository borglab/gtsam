# Install headers
file(GLOB nonlinear_headers "*.h")
install(FILES ${nonlinear_headers} DESTINATION include/gtsam/nonlinear)

# Files to exclude from compilation of tests and timing scripts
set(nonlinear_excluded_files
# "${CMAKE_CURRENT_SOURCE_DIR}/tests/testTypedDiscreteFactor.cpp" # Example of excluding a test     
    ""  # Add to this list, with full path, to exclude
)

# Build tests
if (GTSAM_BUILD_TESTS)
    gtsam_add_subdir_tests(nonlinear "${gtsam-default}" "${gtsam-default}" "${nonlinear_excluded_files}") 
endif(GTSAM_BUILD_TESTS)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_subdir_timing(nonlinear "${gtsam-default}" "${gtsam-default}" "${nonlinear_excluded_files}") 
endif(GTSAM_BUILD_TIMING)

