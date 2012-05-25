# Install headers
file(GLOB base_headers "*.h")
install(FILES ${base_headers} DESTINATION include/gtsam_unstable/base)

# Components to link tests in this subfolder against
set(base_local_libs 
    base
    base_unstable)

set (base_full_libs
    gtsam-static
    gtsam_unstable-static)

# Exclude tests that don't work
set (base_excluded_tests "")

# Add all tests
gtsam_add_subdir_tests(base_unstable "${base_local_libs}" "${base_full_libs}" "${base_excluded_tests}") 
add_dependencies(check.unstable check.base_unstable)
