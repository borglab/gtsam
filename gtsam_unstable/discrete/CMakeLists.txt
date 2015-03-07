# Install headers
file(GLOB discrete_headers "*.h")
install(FILES ${discrete_headers} DESTINATION include/gtsam_unstable/discrete)

# Add all tests
add_subdirectory(tests)

# Add examples
add_subdirectory(examples)
