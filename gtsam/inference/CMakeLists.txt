# Install headers
file(GLOB inference_headers "*.h")
install(FILES ${inference_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/inference)

# Build tests
add_subdirectory(tests)
