# Install headers
file(GLOB base_headers "*.h")
install(FILES ${linear_headers} DESTINATION include/gtsam_unstable/linear)

# Components to link tests in this subfolder against
set(linear_local_libs 
    linear
    linear_unstable
    base)

set (linear_full_libs
    gtsam-static
    gtsam_unstable-static)

# Exclude tests that don't work
set (base_excluded_tests "")

# Add all tests
gtsam_add_subdir_tests(linear_unstable "${linear_local_libs}" "${linear_full_libs}" "${linear_excluded_tests}") 
add_dependencies(check.unstable check.linear_unstable)
