set (excluded_scripts
    elaboratePoint2KalmanFilter.cpp
)
# Add scripts to exclude if GTSAM_USE_BOOST_FEATURES is not set
if (NOT GTSAM_USE_BOOST_FEATURES) 
  # add to excluded scripts
  list (APPEND excluded_scripts
      timeISAM2Chain.cpp
      )
endif()


gtsamAddTimingGlob("*.cpp" excluded_scripts "gtsam")

target_link_libraries(timeGaussianFactorGraph CppUnitLite)
