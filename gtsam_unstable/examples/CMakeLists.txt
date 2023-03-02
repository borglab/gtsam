set (excluded_examples "")

# Add examples to exclude if GTSAM_USE_BOOST_FEATURES is not set
if (NOT GTSAM_USE_BOOST_FEATURES) 
  # add to excluded examples
  list (APPEND excluded_examples
      "GncPoseAveragingExample.cpp"
      )
endif()


gtsamAddExamplesGlob("*.cpp" "${excluded_examples}" "gtsam")
