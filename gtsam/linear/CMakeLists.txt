# Install headers
file(GLOB linear_headers "*.h")
install(FILES ${linear_headers} DESTINATION include/gtsam/linear)

# Components to link tests in this subfolder against
set(linear_local_libs
   linear
   inference
   geometry
   base
   ccolamd
)

# Build tests
if (GTSAM_BUILD_TESTS)
    gtsam_add_tests(linear "${linear_local_libs}")
endif(GTSAM_BUILD_TESTS)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_timing(linear "${linear_local_libs}")
endif(GTSAM_BUILD_TIMING)
