# Install headers
file(GLOB navigation_headers "*.h")
install(FILES ${navigation_headers} DESTINATION include/gtsam_unstable/navigation)

# Components to link tests in this subfolder against
set(navigation_local_libs 
#   navigation_unstable
   slam
   nonlinear
   linear
   inference
   geometry
   base
   ccolamd
)

set (navigation_full_libs
    gtsam-static
    gtsam_unstable-static)

# Exclude tests that don't work
set (navigation_excluded_tests "")

# Add all tests
gtsam_add_subdir_tests(navigation_unstable "${navigation_local_libs}" "${navigation_full_libs}" "${navigation_excluded_tests}") 
add_dependencies(check.unstable check.navigation_unstable)
