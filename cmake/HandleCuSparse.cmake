###############################################################################
# Find CUDA
find_package(CUDAToolkit)

if(CUDAToolkit_FOUND AND GTSAM_WITH_CUSPARSE)
  list(APPEND GTSAM_ADDITIONAL_LIBRARIES CUDA::cusparse CUDA::cusolver CUDA::cudart)
  set(GTSAM_USE_CUSPARSE 1)
endif()
