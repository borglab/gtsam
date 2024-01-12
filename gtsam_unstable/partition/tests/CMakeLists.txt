set(ignore_test "testNestedDissection.cpp")

if (NOT GTSAM_USE_BOOST_FEATURES OR MSVC)
	list(APPEND ignore_test "testFindSeparator.cpp")
endif()

gtsamAddTestsGlob(partition "test*.cpp" "${ignore_test}" "gtsam_unstable;gtsam;metis-gtsam-if")
