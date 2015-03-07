# Install headers
set (slam_excluded_headers #"")
    "${CMAKE_CURRENT_SOURCE_DIR}/serialization.h"
)

file(GLOB slam_headers "*.h")
install(FILES ${slam_headers} DESTINATION include/gtsam/slam)

# Build tests
add_subdirectory(tests)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_subdir_timing(slam "gtsam" "gtsam" "${slam_excluded_files}") 
endif(GTSAM_BUILD_TIMING)
