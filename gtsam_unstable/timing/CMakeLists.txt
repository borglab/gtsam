set(excluded_timings "")
if(NOT GTSAM_ALLOW_DEPRECATED_SINCE_V43)
  list(APPEND excluded_timings "timeInertialNavFactor_GlobalVelocity.cpp")
endif()

gtsamAddTimingGlob("*.cpp" "${excluded_timings}" "gtsam_unstable" ${GTSAM_BUILD_TIMING_ALWAYS})

target_link_libraries(timeHybridAllDiff gtsam_timing_utils)
