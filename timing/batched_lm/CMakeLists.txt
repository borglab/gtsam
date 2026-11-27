if(GTSAM_ENABLE_CUDA)
  gtsamAddTimingGlob("*.cpp" "" "gtsam;gtsam_timing_utils;Threads::Threads"
                    ${GTSAM_BUILD_TIMING_ALWAYS})
endif()
