# Install headers
file(GLOB symbolic_headers "*.h")
install(FILES ${symbolic_headers} DESTINATION include/gtsam/symbolic)

# Build tests
add_subdirectory(tests)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_subdir_timing(symbolic "gtsam" "gtsam" "${symbolic_excluded_files}") 
endif(GTSAM_BUILD_TIMING)

