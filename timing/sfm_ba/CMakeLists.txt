if(GTSAM_ENABLE_CUDA AND GTSAM_ENABLE_CUDSS)
  gtsamAddTimingGlob("*.cpp" "" "gtsam" ${GTSAM_BUILD_TIMING_ALWAYS})
endif()
