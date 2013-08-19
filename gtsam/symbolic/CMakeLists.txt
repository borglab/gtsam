# Install headers
file(GLOB symbolic_headers "*.h")
install(FILES ${symbolic_headers} DESTINATION include/gtsam/symbolic)

# Files to exclude from compilation of tests and timing scripts
set(symbolic_excluded_files
# "${CMAKE_CURRENT_SOURCE_DIR}/tests/testTypedDiscreteFactor.cpp" # Example of excluding a test     
    ""  # Add to this list, with full path, to exclude
)

# Build tests
if (GTSAM_BUILD_TESTS)
    gtsam_add_subdir_tests(symbolic "${gtsam-default}" "${gtsam-default}" "${symbolic_excluded_files}") 
endif(GTSAM_BUILD_TESTS)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_subdir_timing(symbolic "${gtsam-default}" "${gtsam-default}" "${symbolic_excluded_files}") 
endif(GTSAM_BUILD_TIMING)

