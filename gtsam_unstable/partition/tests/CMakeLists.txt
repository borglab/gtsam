set(ignore_test "testNestedDissection.cpp")
# SuiteSparse comes with its own version of METIS - use that instead to avoid conflicts
if (NOT GTSAM_USE_SUITESPARSE)
    gtsamAddTestsGlob(partition "test*.cpp" "${ignore_test}" "gtsam_unstable;gtsam;metis-gtsam-if")
else()
    gtsamAddTestsGlob(partition "test*.cpp" "${ignore_test}" "gtsam_unstable;gtsam;metis")
endif()
