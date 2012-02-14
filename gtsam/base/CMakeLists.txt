# Install headers
file(GLOB base_headers "*.h")
install(FILES ${base_headers} DESTINATION include/gtsam/base)

# Components to link tests in this subfolder against
set(base_local_libs
    base
)

# Build tests
if (GTSAM_BUILD_TESTS)
    gtsam_add_tests(base "${base_local_libs}")
endif(GTSAM_BUILD_TESTS)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_timing(base "${base_local_libs}")
endif(GTSAM_BUILD_TIMING)

