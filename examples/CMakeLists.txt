set (excluded_examples
    DiscreteBayesNet_FG.cpp
    UGM_chain.cpp
    UGM_small.cpp
    elaboratePoint2KalmanFilter.cpp
)

gtsamAddExamplesGlob("*.cpp" "${excluded_examples}" "gtsam;gtsam_unstable;${Boost_PROGRAM_OPTIONS_LIBRARY}")
