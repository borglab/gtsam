# Install headers
file(GLOB constraint_headers "*.h")
install(FILES ${constraint_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/constrained)

# Build tests
add_subdirectory(tests)
