include_directories(${PROJECT_SOURCE_DIR}/gtsam/3rdparty/GeographicLib/include)

gtsamAddTestsGlob(navigation "test*.cpp" "" "gtsam;GeographicLib")
