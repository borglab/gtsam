# Install headers
file(GLOB navigation_headers "*.h")
install(FILES ${navigation_headers} DESTINATION include/gtsam/navigation)

# Add all tests
add_subdirectory(tests)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_subdir_timing(navigation "gtsam" "gtsam" "${navigation_excluded_files}") 
endif(GTSAM_BUILD_TIMING)
