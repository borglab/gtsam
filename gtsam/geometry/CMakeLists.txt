# Install headers
file(GLOB geometry_headers "*.h")
install(FILES ${geometry_headers} DESTINATION include/gtsam/geometry)

# Components to link tests in this subfolder against
set(geometry_local_libs
    base
    geometry
)

# Files to exclude from compilation of tests and timing scripts
set(geometry_excluded_files
# "${CMAKE_CURRENT_SOURCE_DIR}/tests/testTypedDiscreteFactor.cpp" # Example of excluding a test     
    ""  # Add to this list, with full path, to exclude
)

# Build tests
if (GTSAM_BUILD_TESTS)
    gtsam_add_subdir_tests(geometry "${geometry_local_libs}" "gtsam-static" "${geometry_excluded_files}") 
endif(GTSAM_BUILD_TESTS)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_subdir_timing(geometry "${geometry_local_libs}" "gtsam-static" "${geometry_excluded_files}") 
endif(GTSAM_BUILD_TIMING)

