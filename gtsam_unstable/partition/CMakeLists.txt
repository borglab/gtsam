# Install headers
file(GLOB partition_headers "*.h")
install(FILES ${partition_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam_unstable/partition)

add_subdirectory(tests)
