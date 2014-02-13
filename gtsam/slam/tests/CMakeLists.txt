# Files to exclude
set(slam_excluded_files
    testSerialization.cpp
)

gtsamAddTestsGlob(slam "test*.cpp" "${slam_excluded_files}" "gtsam")
