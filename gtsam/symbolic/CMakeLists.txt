# Install headers
file(GLOB symbolic_headers "*.h")
install(FILES ${symbolic_headers} DESTINATION include/gtsam/symbolic)

# Build tests
add_subdirectory(tests)
