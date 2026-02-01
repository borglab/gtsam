# Install headers
file(GLOB constraint_headers "*.h")
install(FILES ${constraint_headers} DESTINATION include/gtsam/constrained)

# Build tests
add_subdirectory(tests)
