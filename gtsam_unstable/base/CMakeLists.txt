# Install headers
file(GLOB base_headers "*.h")
install(FILES ${base_headers} DESTINATION include/gtsam_unstable/base)

# Add all tests
add_subdirectory(tests)
