# Install headers
file(GLOB sfm_headers "*.h")
install(FILES ${sfm_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/sfm)

if(GTSAM_ENABLE_CUDA)
  set(sfm_cuda_headers
      cuda/SfmLevenbergMarquardt.h
      cuda/SfmTypes.h)
  install(FILES ${sfm_cuda_headers}
          DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/sfm/cuda)
endif()

# Build tests
add_subdirectory(tests)
