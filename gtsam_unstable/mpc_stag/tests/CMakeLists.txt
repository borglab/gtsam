set (excluded_tests "")

# Build tests
gtsamAddTestsGlob(mpc_stag_unstable "test*.cpp" "${excluded_tests}" "gtsam_unstable")
