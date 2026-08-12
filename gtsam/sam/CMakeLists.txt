# Install headers
file(GLOB sam_headers "*.h")
install(FILES ${sam_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/sam)

# Build tests
add_subdirectory(tests)
