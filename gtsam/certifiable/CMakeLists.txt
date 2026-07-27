# Install headers
file(GLOB certifiable_headers "*.h")
install(FILES ${certifiable_headers}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/certifiable)

# Build tests
add_subdirectory(tests)
