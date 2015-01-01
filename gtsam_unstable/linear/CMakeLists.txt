# Install headers
file(GLOB linear_headers "*.h")
install(FILES ${linear_headers} DESTINATION include/gtsam_unstable/linear)

# Add all tests
add_subdirectory(tests)
