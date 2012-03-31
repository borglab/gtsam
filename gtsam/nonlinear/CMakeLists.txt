# Install headers
file(GLOB nonlinear_headers "*.h")
install(FILES ${nonlinear_headers} DESTINATION include/gtsam/nonlinear)

# Components to link tests in this subfolder against
set(nonlinear_local_libs
   nonlinear
   linear
   inference
   geometry
   base
   ccolamd
)

# Files to exclude from compilation of tests and timing scripts
set(nonlinear_excluded_files
# "${CMAKE_CURRENT_SOURCE_DIR}/tests/testTypedDiscreteFactor.cpp" # Example of excluding a test     
    ""  # Add to this list, with full path, to exclude
)

# Build tests
if (GTSAM_BUILD_TESTS)
    gtsam_add_subdir_tests(nonlinear "${nonlinear_local_libs}" "gtsam-static" "${nonlinear_excluded_files}") 
endif(GTSAM_BUILD_TESTS)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_subdir_timing(nonlinear "${nonlinear_local_libs}" "gtsam-static" "${nonlinear_excluded_files}") 
endif(GTSAM_BUILD_TIMING)

