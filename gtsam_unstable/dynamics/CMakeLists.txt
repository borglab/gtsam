# Install headers
file(GLOB dynamics_headers "*.h")
install(FILES ${dynamics_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam_unstable/dynamics)

# Add all tests
add_subdirectory(tests)
