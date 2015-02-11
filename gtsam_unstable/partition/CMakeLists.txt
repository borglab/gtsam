# Install headers
file(GLOB partition_headers "*.h")
install(FILES ${partition_headers} DESTINATION include/gtsam_unstable/parition)

add_subdirectory(tests)
