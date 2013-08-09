# Install headers
set (slam_excluded_headers #"")
    "${CMAKE_CURRENT_SOURCE_DIR}/serialization.h"
)

file(GLOB slam_headers "*.h")
list(REMOVE_ITEM slam_headers ${slam_excluded_headers})
install(FILES ${slam_headers} DESTINATION include/gtsam_unstable/slam)

# Components to link tests in this subfolder against
set(slam_local_libs 
   slam_unstable 
   slam
   nonlinear
   linear
   inference
   geometry
   base
   ccolamd
)

set (slam_full_libs
    ${gtsam-default}
    ${gtsam_unstable-default})

# Exclude tests that don't work
set (slam_excluded_tests
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/testAHRS.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/testCombinedImuFactor.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/testImuFactor.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/testInvDepthFactor3.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/testInvDepthFactorVariant1.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/testInvDepthFactorVariant2.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/testInvDepthFactorVariant3.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/testMultiProjectionFactor.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/testSerialization.cpp"
#    ""  # Add to this list, with full path, to exclude
)
# Add all tests
gtsam_add_subdir_tests(slam_unstable "${slam_local_libs}" "${slam_full_libs}" "${slam_excluded_tests}") 
add_dependencies(check.unstable check.slam_unstable)
