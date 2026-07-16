option(GTSAM_ENABLE_CUDA "Enable experimental CUDA acceleration support" OFF)
option(GTSAM_ENABLE_CUDSS "Enable experimental cuDSS linear solver support" OFF)

if(GTSAM_ENABLE_CUDSS AND NOT GTSAM_ENABLE_CUDA)
  message(FATAL_ERROR "GTSAM_ENABLE_CUDSS=ON requires GTSAM_ENABLE_CUDA=ON")
endif()

if(GTSAM_ENABLE_CUDA AND CMAKE_VERSION VERSION_LESS "3.17")
  message(FATAL_ERROR "GTSAM_ENABLE_CUDA requires CMake 3.17 or newer")
endif()

if(GTSAM_ENABLE_CUDA)
  enable_language(CUDA)
  find_package(CUDAToolkit REQUIRED)

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
