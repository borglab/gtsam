# Install headers
file(GLOB dynamics_headers "*.h")
install(FILES ${dynamics_headers} DESTINATION include/gtsam2/dynamics)

# Components to link tests in this subfolder against
set(dynamics_local_libs 
    dynamics)

set (dynamics_full_libs
    gtsam2-static)

# Exclude tests that don't work
set (dynamics_excluded_tests "")

# Add all tests
gtsam_add_subdir_tests(dynamics "${dynamics_local_libs}" "${dynamics_full_libs}" "${dynamics_excluded_tests}") 

