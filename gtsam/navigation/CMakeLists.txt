# Install headers
file(GLOB navigation_headers "*.h")
install(FILES ${navigation_headers} DESTINATION include/gtsam/navigation)

# Add all tests
add_subdirectory(tests)
