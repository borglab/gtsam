# Install headers
file(GLOB base_headers "*.h")
install(FILES ${base_headers} DESTINATION include/gtsam_unstable/base)

# Add all tests
add_subdirectory(tests)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_subdir_timing(base_unstable "${base_full_libs}" "${base_full_libs}" "${base_excluded_files}") 
endif(GTSAM_BUILD_TIMING)
