# if GTSAM_ENABLE_BOOST_SERIALIZATION is OFF then exclude some tests
if (NOT GTSAM_ENABLE_BOOST_SERIALIZATION)
  # create a semicolon seperated list of files to exclude
  set(EXCLUDE_TESTS "testSerializationNonlinear.cpp")
else()
  set(EXCLUDE_TESTS "")
endif()

if(NOT GTSAM_ENABLE_CUDA AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/testCudaDeviceValues.cpp")
  list(APPEND EXCLUDE_TESTS "testCudaDeviceValues.cpp")
endif()

# This optional Ceres test is configured alongside the Ceres timing targets.
list(APPEND EXCLUDE_TESTS "testAdaptAutoDiff.cpp")

gtsamAddTestsGlob(nonlinear "test*.cpp" "${EXCLUDE_TESTS}" "gtsam")
