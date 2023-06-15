# if GTSAM_USE_BOOST_FEATURES is OFF then exclude some tests
if (NOT GTSAM_USE_BOOST_FEATURES)
	# create a semicolon seperated list of files to exclude
	set(EXCLUDE_TESTS "testQPSolver.cpp")
	message(STATUS "Excluding ${EXCLUDE_TESTS}")
else()
	set(EXCLUDE_TESTS "${EXCLUDE_TESTS}")
endif()

gtsamAddTestsGlob(linear_unstable "test*.cpp" "${EXCLUDE_TESTS}" "gtsam_unstable")
