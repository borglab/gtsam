# Install headers
set (slam_excluded_headers #"")
    "${CMAKE_CURRENT_SOURCE_DIR}/serialization.h"
)

file(GLOB slam_headers "*.h")
list(REMOVE_ITEM slam_headers ${slam_excluded_headers})
install(FILES ${slam_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam_unstable/slam)

# Add all tests
add_subdirectory(tests)
