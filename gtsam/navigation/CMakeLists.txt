# Install headers
file(GLOB navigation_headers "*.h")
install(FILES ${navigation_headers} DESTINATION include/gtsam/navigation)

# Exclude tests that don't work
set (navigation_excluded_tests "")

# Add all tests
if (GTSAM_BUILD_TESTS)
	gtsam_add_subdir_tests(navigation "${gtsam-default}" "${gtsam-default}" "${navigation_excluded_tests}") 
endif()

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_subdir_timing(navigation "${gtsam-default}" "${gtsam-default}" "${navigation_excluded_files}") 
endif(GTSAM_BUILD_TIMING)
