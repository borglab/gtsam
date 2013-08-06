# Install headers
file(GLOB navigation_headers "*.h")
install(FILES ${navigation_headers} DESTINATION include/gtsam/navigation)

# Components to link tests in this subfolder against
set(navigation_local_libs 
   nonlinear
   linear
   inference
   geometry
   base
   ccolamd
)

# Exclude tests that don't work
set (navigation_excluded_tests "")

# Add all tests
if (GTSAM_BUILD_TESTS)
	gtsam_add_subdir_tests(navigation "${navigation_local_libs}" "${gtsam-default}" "${navigation_excluded_tests}") 
endif()

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_subdir_timing(navigation "${navigation_local_libs}" "${gtsam-default}" "${navigation_excluded_files}") 
endif(GTSAM_BUILD_TIMING)
