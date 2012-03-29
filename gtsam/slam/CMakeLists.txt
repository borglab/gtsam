# Install headers
file(GLOB slam_headers "*.h")
install(FILES ${slam_headers} DESTINATION include/gtsam/slam)

# Components to link tests in this subfolder against
set(slam_local_libs
   slam
   nonlinear
   linear
   inference
   geometry
   base
   ccolamd
)

# Files to exclude from compilation of tests and timing scripts
set(slam_excluded_files
# "${CMAKE_CURRENT_SOURCE_DIR}/tests/testTypedDiscreteFactor.cpp" # Example of excluding a test     
    ""  # Add to this list, with full path, to exclude
)

# Build tests
if (GTSAM_BUILD_TESTS)
    gtsam_add_subdir_tests(slam "${slam_local_libs}" "gtsam-static" "${slam_excluded_files}") 
endif(GTSAM_BUILD_TESTS)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_subdir_timing(slam "${slam_local_libs}" "gtsam-static" "${slam_excluded_files}") 
endif(GTSAM_BUILD_TIMING)
