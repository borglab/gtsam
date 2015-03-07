# Install headers
file(GLOB inference_headers "*.h")
install(FILES ${inference_headers} DESTINATION include/gtsam/inference)

# Build tests
add_subdirectory(tests)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_subdir_timing(inference "gtsam" "gtsam" "${inference_excluded_files}") 
endif(GTSAM_BUILD_TIMING)

