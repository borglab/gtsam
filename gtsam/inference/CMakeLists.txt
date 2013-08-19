# Install headers
file(GLOB inference_headers "*.h")
install(FILES ${inference_headers} DESTINATION include/gtsam/inference)

# Files to exclude from compilation of tests and timing scripts
set(inference_excluded_files
# "${CMAKE_CURRENT_SOURCE_DIR}/tests/testTypedDiscreteFactor.cpp" # Example of excluding a test     
    ""  # Add to this list, with full path, to exclude
)

# Build tests
if (GTSAM_BUILD_TESTS)
    gtsam_add_subdir_tests(inference "${gtsam-default}" "${gtsam-default}" "${inference_excluded_files}") 
endif(GTSAM_BUILD_TESTS)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_subdir_timing(inference "${gtsam-default}" "${gtsam-default}" "${inference_excluded_files}") 
endif(GTSAM_BUILD_TIMING)

