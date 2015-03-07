# Install headers
file(GLOB nonlinear_headers "*.h")
install(FILES ${nonlinear_headers} DESTINATION include/gtsam_unstable/nonlinear)

# Components to link tests in this subfolder against
set(nonlinear_local_libs 
   nonlinear_unstable
   nonlinear
   linear
   inference
   geometry
   base
   ccolamd
)

set (nonlinear_full_libs
    gtsam-static
    gtsam_unstable-static)

# Exclude tests that don't work
set (nonlinear_excluded_tests "")

# Add all tests
gtsam_add_subdir_tests(nonlinear_unstable "${nonlinear_local_libs}" "${nonlinear_full_libs}" "${nonlinear_excluded_tests}") 
add_dependencies(check.unstable check.nonlinear_unstable)
