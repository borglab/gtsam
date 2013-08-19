# Install headers
file(GLOB base_headers "*.h")
install(FILES ${base_headers} DESTINATION include/gtsam_unstable/base)

set (base_full_libs
    ${gtsam-default}
    ${gtsam_unstable-default})

# Exclude tests that don't work
set (base_excluded_tests "")

# Add all tests
gtsam_add_subdir_tests(base_unstable "${base_full_libs}" "${base_full_libs}" "${base_excluded_tests}") 
add_dependencies(check.unstable check.base_unstable)
