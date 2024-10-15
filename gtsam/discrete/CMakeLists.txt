# Install headers
set(subdir discrete)
file(GLOB discrete_headers "*.h")
install(FILES ${discrete_headers} DESTINATION include/gtsam/discrete)

# Add all tests
add_subdirectory(tests)
