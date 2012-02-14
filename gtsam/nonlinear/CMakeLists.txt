# Install headers
file(GLOB nonlinear_headers "*.h")
install(FILES ${nonlinear_headers} DESTINATION include/gtsam/nonlinear)

# Components to link tests in this subfolder against
set(nonlinear_local_libs
   nonlinear
   linear
   inference
   geometry
   base
   ccolamd
)

# Build tests
if (GTSAM_BUILD_TESTS)
    gtsam_add_tests(nonlinear "${nonlinear_local_libs}")
endif(GTSAM_BUILD_TESTS)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_timing(nonlinear "${nonlinear_local_libs}")
endif(GTSAM_BUILD_TIMING)

