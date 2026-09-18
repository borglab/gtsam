set(EXCLUDE_TESTS "")

if(NOT GTSAM_ENABLE_CUDA AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/testCudaSfm.cpp")
  list(APPEND EXCLUDE_TESTS "testCudaSfm.cpp")
endif()

gtsamAddTestsGlob(sfm "test*.cpp" "${EXCLUDE_TESTS}" "gtsam")
