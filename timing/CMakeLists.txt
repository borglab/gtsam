list(APPEND to_exclude
    timeFactorOverhead.cpp
    timeSLAMlike.cpp)
gtsamAddTimingGlob("*.cpp" "${to_exclude}" "gtsam")

target_link_libraries(timeGaussianFactorGraph CppUnitLite)
