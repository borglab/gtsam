# Install headers
file(GLOB linear_headers "*.h")
install(FILES ${linear_headers} DESTINATION include/gtsam/linear)

# Build tests
add_subdirectory(tests)
