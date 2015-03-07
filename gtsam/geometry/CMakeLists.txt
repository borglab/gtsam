# Install headers
file(GLOB geometry_headers "*.h")
install(FILES ${geometry_headers} DESTINATION include/gtsam/geometry)

# Files to exclude from compilation of tests and timing scripts
set(geometry_excluded_files
# "${CMAKE_CURRENT_SOURCE_DIR}/tests/testTypedDiscreteFactor.cpp" # Example of excluding a test     
    ""  # Add to this list, with full path, to exclude
)

# Build tests
if (GTSAM_BUILD_TESTS)
    gtsam_add_subdir_tests(geometry "${gtsam-default}" "${gtsam-default}" "${geometry_excluded_files}") 
endif(GTSAM_BUILD_TESTS)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_subdir_timing(geometry "${gtsam-default}" "${gtsam-default}" "${geometry_excluded_files}") 
endif(GTSAM_BUILD_TIMING)

