# Install headers
file(GLOB gnssNavigation_headers "*.h")
install(FILES ${gnssNavigation_headers} DESTINATION include/gtsam/gnssNavigation)

# Add all tests
add_subdirectory(tests)
