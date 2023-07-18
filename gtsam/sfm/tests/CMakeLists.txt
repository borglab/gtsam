set(tests_excluded testMFAS.cpp testBinaryMeasurement.cpp)

gtsamAddTestsGlob(sfm "test*.cpp" "${tests_excluded}" "gtsam")
gtsamAddTestsGlob(sfm2 "testBinaryMeasurement.cpp testMFAS.cpp" "" "gtsam")
