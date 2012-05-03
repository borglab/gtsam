# Install headers
file(GLOB slam_headers "*.h")
install(FILES ${slam_headers} DESTINATION include/gtsam2/slam)

# Components to link tests in this subfolder against
set(slam_local_libs 
    slam)

set (slam_full_libs
    gtsam2-static)

# Exclude tests that don't work
set (slam_excluded_tests "")

# Add all tests
gtsam_add_subdir_tests(slam "${slam_local_libs}" "${slam_full_libs}" "${slam_excluded_tests}") 

