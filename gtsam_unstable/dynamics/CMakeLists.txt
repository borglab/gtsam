# Install headers
file(GLOB dynamics_headers "*.h")
install(FILES ${dynamics_headers} DESTINATION include/gtsam_unstable/dynamics)

# Add all tests
add_subdirectory(tests)
