# Install headers
file(GLOB nonlinear_headers "*.h")
install(FILES ${nonlinear_headers} DESTINATION include/gtsam/nonlinear)

# Build tests
add_subdirectory(tests)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_subdir_timing(nonlinear "gtsam" "gtsam" "${nonlinear_excluded_files}") 
endif(GTSAM_BUILD_TIMING)

