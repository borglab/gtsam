# Install headers
file(GLOB discrete_headers "*.h")
install(FILES ${discrete_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam_unstable/discrete)

# Add all tests
add_subdirectory(tests)

# Add examples
add_subdirectory(examples)
