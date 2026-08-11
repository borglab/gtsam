# Install headers
file(GLOB geometry_headers "*.h")
install(FILES ${geometry_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam_unstable/geometry)

# Add all tests
add_subdirectory(tests)
