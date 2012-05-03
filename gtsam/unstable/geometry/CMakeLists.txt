# Install headers
file(GLOB geometry_headers "*.h")
install(FILES ${geometry_headers} DESTINATION include/gtsam2/geometry)

# Components to link tests in this subfolder against
set(geometry_local_libs 
    geometry)

set (geometry_full_libs
    gtsam2-static)

# Exclude tests that don't work
set (geometry_excluded_tests "")

# Add all tests
gtsam_add_subdir_tests(geometry "${geometry_local_libs}" "${geometry_full_libs}" "${geometry_excluded_tests}") 

