# Install headers
file(GLOB linear_headers "*.h")
install(FILES ${linear_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/linear)
file(GLOB linear_internal_headers "internal/*.h")
install(FILES ${linear_internal_headers}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/linear/internal)
if(GTSAM_ENABLE_CUDA)
  set(linear_cuda_headers
      cuda/DeviceSparseSpdSystem.h
      cuda/LinearSolver.h
      cuda/LinearSystem.h)
  install(FILES ${linear_cuda_headers}
          DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/linear/cuda)
endif()

# Build tests
add_subdirectory(tests)
