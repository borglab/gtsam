# Install headers
file(GLOB slam_headers "*.h")
install(FILES ${slam_headers} DESTINATION include/gtsam_unstable/slam)

# Components to link tests in this subfolder against
set(slam_local_libs 
   slam_unstable 
   slam
   nonlinear
   linear
   inference
   geometry
   base
   ccolamd
)

set (slam_full_libs
    gtsam-static
    gtsam_unstable-static)

# Exclude tests that don't work
set (slam_excluded_tests "")

# Add all tests
gtsam_add_subdir_tests(slam_unstable "${slam_local_libs}" "${slam_full_libs}" "${slam_excluded_tests}") 
add_dependencies(check.unstable check.slam_unstable)
