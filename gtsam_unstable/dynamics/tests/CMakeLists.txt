set (excluded_tests "")

# TODO(dellaert): these segfault, and are rather obsolete, so we stop compiling them:
list(APPEND excluded_tests "testIMUSystem.cpp" "testPoseRTV.cpp")

# Build tests
gtsamAddTestsGlob(dynamics_unstable "test*.cpp" "${excluded_tests}" "gtsam_unstable")
