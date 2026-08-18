set (excluded_tests "")

# Build tests
gtsamAddTestsGlob(dynamics_unstable "test*.cpp" "${excluded_tests}" "gtsam_unstable")
