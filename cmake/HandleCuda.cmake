option(GTSAM_ENABLE_CUDA "Enable experimental CUDA acceleration support" OFF)
option(GTSAM_ENABLE_CUDSS "Enable experimental cuDSS linear solver support" OFF)

if(GTSAM_ENABLE_CUDSS AND NOT GTSAM_ENABLE_CUDA)
  message(FATAL_ERROR "GTSAM_ENABLE_CUDSS=ON requires GTSAM_ENABLE_CUDA=ON")
endif()

if(GTSAM_ENABLE_CUDA AND CMAKE_VERSION VERSION_LESS "3.17")
  message(FATAL_ERROR "GTSAM_ENABLE_CUDA requires CMake 3.17 or newer")
endif()

# The CUDA kernels accumulate into double-precision buffers with atomicAdd,
# which requires compute capability 6.0 (Pascal, 2016) or newer.
set(GTSAM_CUDA_MIN_ARCHITECTURE 60)

if(GTSAM_ENABLE_CUDA)
  # Without this, CMP0104 leaves CMAKE_CUDA_ARCHITECTURES at nvcc's own default,
  # which is 52 through CUDA 12 and therefore cannot compile the kernels. Choose
  # a default before enable_language(CUDA) so it is not overwritten. "native"
  # builds only for the GPUs present, so name the architectures explicitly when
  # producing binaries for other machines.
  if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
    if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24")
      set(CMAKE_CUDA_ARCHITECTURES "native"
          CACHE STRING "CUDA architectures GTSAM generates code for")
    else()
      set(CMAKE_CUDA_ARCHITECTURES "${GTSAM_CUDA_MIN_ARCHITECTURE}"
          CACHE STRING "CUDA architectures GTSAM generates code for")
    endif()
  endif()

  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)

  # Catch an unusable architecture, including one inherited from a build
  # directory configured before the default above existed.
  foreach(gtsam_cuda_arch IN LISTS CMAKE_CUDA_ARCHITECTURES)
    if(gtsam_cuda_arch MATCHES "^([0-9]+)")
      if(CMAKE_MATCH_1 LESS GTSAM_CUDA_MIN_ARCHITECTURE)
        message(FATAL_ERROR
          "GTSAM_ENABLE_CUDA requires compute capability "
          "${GTSAM_CUDA_MIN_ARCHITECTURE} or newer, but CMAKE_CUDA_ARCHITECTURES "
          "is '${CMAKE_CUDA_ARCHITECTURES}'. The kernels use double-precision "
          "atomicAdd, which older architectures do not provide. Reconfigure "
          "with -DCMAKE_CUDA_ARCHITECTURES=native, or name the architectures "
          "you are targeting.")
      endif()
    endif()
  endforeach()

  message(STATUS "GTSAM CUDA architectures: ${CMAKE_CUDA_ARCHITECTURES}")

  set(CMAKE_CUDA_STANDARD 17)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  set(CMAKE_CUDA_EXTENSIONS OFF)

  list_append_cache(GTSAM_COMPILE_DEFINITIONS_PUBLIC GTSAM_ENABLE_CUDA=1)

  if(GTSAM_ENABLE_CUDSS)
    if(NOT TARGET cudss::cudss)
      find_path(CUDSS_INCLUDE_DIR cudss.h
        HINTS
          ${CUDAToolkit_INCLUDE_DIRS}
          $ENV{CUDA_HOME}/include
          $ENV{CONDA_PREFIX}/include
        PATH_SUFFIXES
          libcudss/13
          libcudss/12
      )

      find_library(CUDSS_LIBRARY cudss
        HINTS
          ${CUDAToolkit_LIBRARY_DIR}
          ${CUDAToolkit_LIBRARY_ROOT}
          $ENV{CUDA_HOME}/lib64
          $ENV{CONDA_PREFIX}/lib
        PATH_SUFFIXES
          libcudss/13
          libcudss/12
      )

      if(NOT CUDSS_INCLUDE_DIR)
        message(FATAL_ERROR "GTSAM_ENABLE_CUDSS=ON but cudss.h was not found")
      endif()

      if(NOT CUDSS_LIBRARY)
        message(FATAL_ERROR "GTSAM_ENABLE_CUDSS=ON but libcudss was not found")
      endif()

      add_library(cudss::cudss UNKNOWN IMPORTED)
      set_target_properties(cudss::cudss PROPERTIES
        IMPORTED_LOCATION "${CUDSS_LIBRARY}"
        INTERFACE_INCLUDE_DIRECTORIES "${CUDSS_INCLUDE_DIR};${CUDAToolkit_INCLUDE_DIRS}")
    endif()
    list_append_cache(GTSAM_COMPILE_DEFINITIONS_PUBLIC GTSAM_ENABLE_CUDSS=1)
  endif()
endif()
