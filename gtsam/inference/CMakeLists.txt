# Install headers
file(GLOB inference_headers "*.h")
install(FILES ${inference_headers} DESTINATION include/gtsam/inference)

# Components to link tests in this subfolder against
set(inference_local_libs
   inference
   geometry
   base
   ccolamd
)

# Files to exclude from compilation of tests and timing scripts
set(inference_excluded_files
# "${CMAKE_CURRENT_SOURCE_DIR}/tests/testTypedDiscreteFactor.cpp" # Example of excluding a test     
    ""  # Add to this list, with full path, to exclude
)

# Build tests
if (GTSAM_BUILD_TESTS)
    gtsam_add_subdir_tests(inference "${inference_local_libs}" "gtsam-static" "${inference_excluded_files}") 
endif(GTSAM_BUILD_TESTS)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_subdir_timing(inference "${inference_local_libs}" "gtsam-static" "${inference_excluded_files}") 
endif(GTSAM_BUILD_TIMING)

