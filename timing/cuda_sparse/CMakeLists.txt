if(GTSAM_ENABLE_CUDA AND GTSAM_ENABLE_CUDSS)
  gtsamAddTimingGlob("*.cpp" "" "gtsam" ${GTSAM_BUILD_TIMING_ALWAYS})
  target_compile_definitions(
    timeCudaSparseLM
    PRIVATE GTSAM_BENCHMARK_BUILD_TYPE="${CMAKE_BUILD_TYPE}")
endif()
