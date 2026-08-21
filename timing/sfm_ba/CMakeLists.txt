if(GTSAM_ENABLE_CUDA)
gtsamAddTimingGlob("*.cpp" "" "gtsam;gtsam_timing_utils"
                  ${GTSAM_BUILD_TIMING_ALWAYS})
endif()
