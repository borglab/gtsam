# Install headers
file(GLOB base_headers "*.h")
install(FILES ${base_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam_unstable/base)

# Add all tests
add_subdirectory(tests)
