# Install headers
file(GLOB partition_headers "*.h")
install(FILES ${partition_headers} DESTINATION include/gtsam_unstable/parition)

set(ignore_test "tests/testNestedDissection.cpp")
# Add all tests
gtsamAddTestsGlob(partition_unstable "tests/*.cpp" "${ignore_test}" "gtsam_unstable;metis")
