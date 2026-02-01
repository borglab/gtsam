# Install headers
file(GLOB sfm_headers "*.h")
install(FILES ${sfm_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/sfm)

# Build tests
add_subdirectory(tests)
