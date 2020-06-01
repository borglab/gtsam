# Install headers
file(GLOB gpstk_headers "*.hpp")
install(FILES ${gpstk_headers} DESTINATION include/gtsam/gpstk)

# Add all tests
add_subdirectory(tests)
