set (excluded_examples
    elaboratePoint2KalmanFilter.cpp
)

# if GTSAM_ENABLE_BOOST_SERIALIZATION is not set then SolverComparer.cpp will not compile
if (NOT GTSAM_ENABLE_BOOST_SERIALIZATION)
    list (APPEND excluded_examples
    SolverComparer.cpp
  )
endif()

# Add examples to exclude if GTSAM_USE_BOOST is not set
if (NOT GTSAM_USE_BOOST) 
  # add to excluded examples
  list (APPEND excluded_examples
      # Boost examples
      SolverComparer.cpp
      )
endif()

gtsamAddExamplesGlob("*.cpp" "${excluded_examples}" "gtsam;gtsam_unstable;${Boost_PROGRAM_OPTIONS_LIBRARY}")
