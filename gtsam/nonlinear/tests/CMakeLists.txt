# if GTSAM_ENABLE_BOOST_SERIALIZATION is OFF then exclude some tests
if (NOT GTSAM_ENABLE_BOOST_SERIALIZATION)
  # create a semicolon seperated list of files to exclude
  set(EXCLUDE_TESTS "testSerializationNonlinear.cpp")
else()
  set(EXCLUDE_TESTS "")
endif()

if(NOT GTSAM_ENABLE_CUDA)
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/testCudaDeviceValues.cpp")
    list(APPEND EXCLUDE_TESTS "testCudaDeviceValues.cpp")
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/testCudaSparseJacobian.cpp")
    list(APPEND EXCLUDE_TESTS "testCudaSparseJacobian.cpp")
  endif()
  if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/testCudaSparseLevenbergMarquardt.cpp")
    list(APPEND EXCLUDE_TESTS "testCudaSparseLevenbergMarquardt.cpp")
  endif()
endif()

gtsamAddTestsGlob(nonlinear "test*.cpp" "${EXCLUDE_TESTS}" "gtsam")
