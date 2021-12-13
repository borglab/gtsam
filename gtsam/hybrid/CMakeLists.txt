# Install headers
file(GLOB hybrid_headers "*.h")
install(FILES ${hybrid_headers} DESTINATION include/gtsam/hybrid)

# Build tests
add_subdirectory(tests)
