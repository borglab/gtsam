# Install headers
file(GLOB slam_headers "*.h")
install(FILES ${slam_headers} DESTINATION include/gtsam/slam)

# Components to link tests in this subfolder against
set(slam_local_libs
   slam
   nonlinear
   linear
   inference
   geometry
   base
   ccolamd
)

# Build tests
if (GTSAM_BUILD_TESTS)
    gtsam_add_tests(slam "${slam_local_libs}")
endif(GTSAM_BUILD_TESTS)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_timing(slam "${slam_local_libs}")
endif(GTSAM_BUILD_TIMING)
