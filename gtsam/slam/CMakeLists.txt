# Install headers
set (slam_excluded_headers #"")
    "${CMAKE_CURRENT_SOURCE_DIR}/serialization.h"
)

file(GLOB slam_headers "*.h")
install(FILES ${slam_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/slam)

if(GTSAM_ENABLE_CUDA)
  file(GLOB slam_cuda_headers "cuda/*.h")
  install(FILES ${slam_cuda_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/slam/cuda)
endif()

# Build tests
add_subdirectory(tests)
