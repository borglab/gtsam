# Install headers
file(GLOB base_headers "*.h")
install(FILES ${base_headers} DESTINATION include/gtsam/base)

file(GLOB base_headers_tree "treeTraversal/*.h")
install(FILES ${base_headers_tree} DESTINATION include/gtsam/base/treeTraversal)

# Files to exclude from compilation of tests and timing scripts
set(base_excluded_files
# "${CMAKE_CURRENT_SOURCE_DIR}/tests/testTypedDiscreteFactor.cpp" # Example of excluding a test
#    ""  # Add to this list, with full path, to exclude
)

# Build tests
if (GTSAM_BUILD_TESTS)
    gtsam_add_subdir_tests(base "gtsam" "gtsam" "${base_excluded_files}") 
endif(GTSAM_BUILD_TESTS)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_subdir_timing(base "gtsam" "gtsam" "${base_excluded_files}") 
endif(GTSAM_BUILD_TIMING)

