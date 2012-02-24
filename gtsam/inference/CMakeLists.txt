# Install headers
file(GLOB inference_headers "*.h")
install(FILES ${inference_headers} DESTINATION include/gtsam/inference)

# Components to link tests in this subfolder against
set(inference_local_libs
   inference
   geometry
   base
   ccolamd
)

# Build tests
if (GTSAM_BUILD_TESTS)
    gtsam_add_tests(inference "${inference_local_libs}")
endif(GTSAM_BUILD_TESTS)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_timing(inference "${inference_local_libs}")
endif(GTSAM_BUILD_TIMING)

