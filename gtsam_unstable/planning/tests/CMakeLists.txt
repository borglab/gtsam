# Authors: Frank Dellaert and Akash Ratheesh

set (excluded_tests "")

# Build tests
gtsamAddTestsGlob(planning_unstable "test*.cpp" "${excluded_tests}" "gtsam_unstable")
