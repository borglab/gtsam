# Install headers
file(GLOB geometry_headers "*.h")
install(FILES ${geometry_headers} DESTINATION include/gtsam/geometry)

# Build tests
add_subdirectory(tests)
