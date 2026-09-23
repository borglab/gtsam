
# Exclude tests that don't work
set (slam_excluded_tests
    testSerialization.cpp
)

gtsamAddTestsGlob(slam_unstable "test*.cpp" "${slam_excluded_tests}" "gtsam_unstable")
