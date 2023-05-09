set (excluded_examples
    # fileToExclude.cpp
)

gtsamAddExamplesGlob("*.cpp" "${excluded_examples}" "gtsam_unstable")
