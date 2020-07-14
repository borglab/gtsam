set(ignore_test "testNestedDissection.cpp")
gtsamAddTestsGlob(partition "test*.cpp" "${ignore_test}" "gtsam_unstable;gtsam;metis-gtsam")
