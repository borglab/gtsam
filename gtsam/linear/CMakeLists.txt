# Install headers
file(GLOB linear_headers "*.h")
install(FILES ${linear_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/linear)

# Build tests
add_subdirectory(tests)
