# Install headers
file(GLOB base_headers "*.h")
install(FILES ${base_headers} DESTINATION include/gtsam2/base)

# Components to link tests in this subfolder against
set(base_local_libs 
    base)

set (base_full_libs
    gtsam2-static)

# Exclude tests that don't work
set (base_excluded_tests "")

# Add all tests
gtsam_add_subdir_tests(base "${base_local_libs}" "${base_full_libs}" "${base_excluded_tests}") 

