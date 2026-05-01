set(excluded_tests "")

gtsamAddTestsGlob(navigation_unstable "test*.cpp" "${excluded_tests}" "gtsam_unstable")
