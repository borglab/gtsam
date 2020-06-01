# Install headers
file(GLOB robustModels_headers "*.h")
install(FILES ${robustModels_headers} DESTINATION include/gtsam/robustModels)

# Add all tests
add_subdirectory(tests)
