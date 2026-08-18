if(GTSAM_ENABLE_CUDA)
  gtsamAddTimingGlob("*.cpp" "" "gtsam;gtsam_timing_utils"
                    ${GTSAM_BUILD_TIMING_ALWAYS})
  target_compile_definitions(
    timeCudaSparseLM
    PRIVATE GTSAM_BENCHMARK_BUILD_TYPE="${CMAKE_BUILD_TYPE}")
endif()
