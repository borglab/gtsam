# Install headers
file(GLOB configReader_headers "*.h*")
install(FILES ${configReader_headers} DESTINATION include/gtsam/configReader)

# Add all tests
add_subdirectory(tests)
