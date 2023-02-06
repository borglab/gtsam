# disable tests if GTSAM_USE_BOOST_FEATURES is OFF
if (NOT GTSAM_USE_BOOST_FEATURES)
  set (excluded_examples
      # fileToExclude.cpp
      "schedulingExample.cpp"
      "schedulingQuals12.cpp"
      "schedulingQuals13.cpp"
      )
else()
  set (excluded_examples "")
endif()



gtsamAddExamplesGlob("*.cpp" "${excluded_examples}" "gtsam_unstable")
