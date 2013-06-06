# Install headers
file(GLOB symbolic_headers "*.h")
install(FILES ${symbolic_headers} DESTINATION include/gtsam/symbolic)

# Components to link tests in this subfolder against
set(symbolic_local_libs
   inference
   geometry
   base
   ccolamd
   symbolic
)

# Files to exclude from compilation of tests and timing scripts
set(symbolic_excluded_files
# "${CMAKE_CURRENT_SOURCE_DIR}/tests/testTypedDiscreteFactor.cpp" # Example of excluding a test     
    ""  # Add to this list, with full path, to exclude
)

# Build tests
if (GTSAM_BUILD_TESTS)
    gtsam_add_subdir_tests(symbolic "${symbolic_local_libs}" "${gtsam-default}" "${symbolic_excluded_files}") 
endif(GTSAM_BUILD_TESTS)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_subdir_timing(symbolic "${symbolic_local_libs}" "${gtsam-default}" "${symbolic_excluded_files}") 
endif(GTSAM_BUILD_TIMING)

