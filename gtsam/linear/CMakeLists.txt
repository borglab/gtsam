# Install headers
file(GLOB linear_headers "*.h")
install(FILES ${linear_headers} DESTINATION include/gtsam/linear)

# Build tests
add_subdirectory(tests)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_subdir_timing(linear "gtsam" "gtsam" "${linear_excluded_files}") 
endif(GTSAM_BUILD_TIMING)
