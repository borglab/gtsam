# if GTSAM_ENABLE_BOOST_SERIALIZATION is OFF then exclude some tests
if (NOT GTSAM_ENABLE_BOOST_SERIALIZATION)
  # create a semicolon seperated list of files to exclude
  set(EXCLUDE_TESTS "testSerializationNonlinear.cpp")
else()
  set(EXCLUDE_TESTS "")
endif()

gtsamAddTestsGlob(nonlinear "test*.cpp" "${EXCLUDE_TESTS}" "gtsam")
