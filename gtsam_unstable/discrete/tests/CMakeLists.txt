set(excluded_sources "")
if (NOT GTSAM_USE_BOOST_FEATURES)
  list(APPEND excluded_sources "testScheduler.cpp")
endif()

gtsamAddTestsGlob(discrete_unstable "test*.cpp" "${excluded_sources}" "gtsam_unstable")
