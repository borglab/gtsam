# Install headers
file(GLOB geometry_headers "*.h")
install(FILES ${geometry_headers} DESTINATION include/gtsam/geometry)

# Components to link tests in this subfolder against
set(geometry_local_libs
    base
    geometry
)

# Build tests
if (GTSAM_BUILD_TESTS)
    gtsam_add_tests(geometry "${geometry_local_libs}")
endif(GTSAM_BUILD_TESTS)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_timing(geometry "${geometry_local_libs}")
endif(GTSAM_BUILD_TIMING)

