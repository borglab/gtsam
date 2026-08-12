# Install headers
set(subdir hybrid)
file(GLOB hybrid_headers "*.h")
install(FILES ${hybrid_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/hybrid)

# Add all tests
add_subdirectory(tests)
