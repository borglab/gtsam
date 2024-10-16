# Install headers
file(GLOB constraint_headers "*.h")
install(FILES ${constraint_headers} DESTINATION include/gtsam/constraint)

# Build tests
add_subdirectory(tests)
