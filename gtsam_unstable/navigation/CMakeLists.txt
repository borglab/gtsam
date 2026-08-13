# Install headers
file(GLOB navigation_headers "*.h")
install(FILES ${navigation_headers}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam_unstable/navigation)

# Add all tests
add_subdirectory(tests)
