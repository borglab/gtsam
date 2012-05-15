# Install headers
set(subdir discrete)
file(GLOB discrete_headers "*.h")
# FIXME: exclude headers
install(FILES ${discrete_headers} DESTINATION include/gtsam/discrete)

# Set up library dependencies
set (discrete_local_libs 
   discrete
   inference
   base
   ccolamd
)

# Exclude tests that don't work
set (discrete_excluded_tests "")

# Add all tests
if (GTSAM_BUILD_TESTS)
    gtsam_add_subdir_tests(discrete "${discrete_local_libs}" "gtsam-static" "${discrete_excluded_tests}")
endif()

# Build timing scripts
#if (GTSAM_BUILD_TIMING)
#    gtsam_add_timing(discrete "${discrete_local_libs}")
#endif(GTSAM_BUILD_TIMING)

