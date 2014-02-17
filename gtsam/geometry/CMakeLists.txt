# Install headers
file(GLOB geometry_headers "*.h")
install(FILES ${geometry_headers} DESTINATION include/gtsam/geometry)

# Build tests
add_subdirectory(tests)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_subdir_timing(geometry "gtsam" "gtsam" "${geometry_excluded_files}") 
endif(GTSAM_BUILD_TIMING)

