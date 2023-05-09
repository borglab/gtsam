
# Exclude tests that don't work
set (slam_excluded_tests
    testSerialization.cpp
    testSmartStereoProjectionFactorPP.cpp # unstable after PR #1442
)

gtsamAddTestsGlob(slam_unstable "test*.cpp" "${slam_excluded_tests}" "gtsam_unstable")
