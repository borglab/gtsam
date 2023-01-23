set (excluded_examples
    elaboratePoint2KalmanFilter.cpp
)

# if GTSAM_ENABLE_BOOST_SERIALIZATION is not set then SolverComparer.cpp will not compile
if (NOT GTSAM_ENABLE_BOOST_SERIALIZATION)
  set (excluded_examples
    ${excluded_examples}
    SolverComparer.cpp
  )
endif()

gtsamAddExamplesGlob("*.cpp" "${excluded_examples}" "gtsam;gtsam_unstable;${Boost_PROGRAM_OPTIONS_LIBRARY}")
