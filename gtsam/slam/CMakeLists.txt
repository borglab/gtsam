# Install headers
set (slam_excluded_headers #"")
    "${CMAKE_CURRENT_SOURCE_DIR}/serialization.h"
)

file(GLOB slam_headers "*.h")
list(REMOVE_ITEM slam_headers ${slam_excluded_headers})
install(FILES ${slam_headers} DESTINATION include/gtsam/slam)

# Files to exclude from compilation of tests and timing scripts
set(slam_excluded_files
 "${CMAKE_CURRENT_SOURCE_DIR}/tests/testSerialization.cpp"      
#    ""  # Add to this list, with full path, to exclude
)

# Build tests
if (GTSAM_BUILD_TESTS)
    gtsam_add_subdir_tests(slam "${gtsam-default}" "${gtsam-default}" "${slam_excluded_files}") 
endif(GTSAM_BUILD_TESTS)

# Build timing scripts
if (GTSAM_BUILD_TIMING)
    gtsam_add_subdir_timing(slam "${gtsam-default}" "${gtsam-default}" "${slam_excluded_files}") 
endif(GTSAM_BUILD_TIMING)
