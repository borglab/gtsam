
###############################################################################
# Find OpenMP (if we're also using MKL)
find_package(OpenMP)  # do this here to generate correct message if disabled

if(GTSAM_WITH_EIGEN_MKL AND GTSAM_WITH_EIGEN_MKL_OPENMP AND GTSAM_USE_EIGEN_MKL)
    if(OPENMP_FOUND AND GTSAM_USE_EIGEN_MKL AND GTSAM_WITH_EIGEN_MKL_OPENMP)
        set(GTSAM_USE_EIGEN_MKL_OPENMP 1) # This will go into config.h
        list_append_cache(GTSAM_COMPILE_OPTIONS_PUBLIC ${OpenMP_CXX_FLAGS})
    endif()
endif()
