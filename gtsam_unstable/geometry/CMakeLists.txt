# Install headers
file(GLOB geometry_headers "*.h")
install(FILES ${geometry_headers} DESTINATION include/gtsam_unstable/geometry)

# Components to link tests in this subfolder against
set(geometry_local_libs 
   #geometry_unstable # No sources currently, so no convenience lib
   geometry
   base
   ccolamd
)

set (geometry_full_libs
    gtsam-static
    gtsam_unstable-static)

# Exclude tests that don't work
set (geometry_excluded_tests "")

# Add all tests
gtsam_add_subdir_tests(geometry_unstable "${geometry_local_libs}" "${geometry_full_libs}" "${geometry_excluded_tests}") 
add_dependencies(check.unstable check.geometry_unstable)
