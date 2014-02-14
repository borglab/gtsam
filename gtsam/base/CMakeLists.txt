# Install headers
file(GLOB base_headers "*.h")
install(FILES ${base_headers} DESTINATION include/gtsam/base)

file(GLOB base_headers_tree "treeTraversal/*.h")
install(FILES ${base_headers_tree} DESTINATION include/gtsam/base/treeTraversal)

# Build tests
add_subdirectory(tests)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_subdir_timing(base "gtsam" "gtsam" "${base_excluded_files}") 
endif(GTSAM_BUILD_TIMING)

