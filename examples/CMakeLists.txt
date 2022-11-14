set (excluded_examples
    elaboratePoint2KalmanFilter.cpp
)

gtsamAddExamplesGlob("*.cpp" "${excluded_examples}" "gtsam;gtsam_unstable;${Boost_PROGRAM_OPTIONS_LIBRARY}")
