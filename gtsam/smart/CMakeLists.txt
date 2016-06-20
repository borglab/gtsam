# Install headers
file(GLOB smart_headers "*.h")
install(FILES ${smart_headers} DESTINATION include/gtsam/smart)

# Build tests
add_subdirectory(tests)
