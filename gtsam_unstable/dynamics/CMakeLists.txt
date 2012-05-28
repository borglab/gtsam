# Install headers
file(GLOB dynamics_headers "*.h")
install(FILES ${dynamics_headers} DESTINATION include/gtsam_unstable/dynamics)

# Components to link tests in this subfolder against
set(dynamics_local_libs 
   dynamics_unstable
   slam
   nonlinear
   linear
   inference
   geometry
   base
   ccolamd
)

set (dynamics_full_libs
    gtsam-static
    gtsam_unstable-static)

# Exclude tests that don't work
set (dynamics_excluded_tests "")

# Add all tests
gtsam_add_subdir_tests(dynamics_unstable "${dynamics_local_libs}" "${dynamics_full_libs}" "${dynamics_excluded_tests}") 
add_dependencies(check.unstable check.dynamics_unstable)
