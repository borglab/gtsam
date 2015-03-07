# Install headers
set(subdir discrete)
file(GLOB discrete_headers "*.h")
# FIXME: exclude headers
install(FILES ${discrete_headers} DESTINATION include/gtsam/discrete)

# Add all tests
add_subdirectory(tests)

# Build timing scripts
#if (GTSAM_BUILD_TIMING)
#    gtsam_add_timing(discrete "${discrete_local_libs}")
#endif(GTSAM_BUILD_TIMING)

