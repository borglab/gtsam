# Add scripts to exclude if GTSAM_ENABLE_BOOST_SERIALIZATION is not set
if (NOT GTSAM_ENABLE_BOOST_SERIALIZATION) 
  # add to excluded scripts
  list (APPEND excluded_scripts
      "timeIncremental.cpp"
      )
endif()

# Add scripts to exclude if GTSAM_USE_BOOST_FEATURES is not set
if (NOT GTSAM_USE_BOOST_FEATURES) 
  # add to excluded scripts
  list (APPEND excluded_scripts
      "timeISAM2Chain.cpp"
      )
endif()

add_library(gtsam_timing_utils STATIC internal/TimingUtils.cpp
                                      internal/SfmBalBenchmark.cpp
                                      internal/SfmPcgBenchmark.cpp)
target_include_directories(gtsam_timing_utils PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(gtsam_timing_utils PUBLIC gtsam)
gtsam_apply_build_flags(gtsam_timing_utils)
set_property(TARGET gtsam_timing_utils PROPERTY FOLDER "Timing")

gtsamAddTestsGlob(timing_utils "tests/test*.cpp" "" "gtsam_timing_utils")

gtsamAddTimingGlob("*.cpp" "${excluded_scripts}" "gtsam;gtsam_timing_utils" ${GTSAM_BUILD_TIMING_ALWAYS})

target_link_libraries(timeGaussianFactorGraph CppUnitLite)

if(OpenMP_CXX_FOUND)
  target_link_libraries(timePCGSolver OpenMP::OpenMP_CXX)
endif()

add_subdirectory(sfm_ba)
