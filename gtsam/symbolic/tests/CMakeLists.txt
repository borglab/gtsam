# if GTSAM_ENABLE_BOOST_SERIALIZATION is OFF then exclude some tests
if (NOT GTSAM_ENABLE_BOOST_SERIALIZATION)
  # create a semicolon seperated list of files to exclude
  set(EXCLUDE_TESTS "testSerializationSymbolic.cpp")
else()
  set(EXCLUDE_TESTS "")
endif()

list(APPEND EXCLUDE_TESTS testSymbolicConditional.cpp)

gtsamAddTestsGlob(symbolic "test*.cpp" "${EXCLUDE_TESTS}" "gtsam")
gtsamAddTestsGlob(symbolic2 "testSymbolicConditional.cpp" "" "gtsam")
