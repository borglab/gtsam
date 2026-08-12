# Install headers
set(subdir discrete)
file(GLOB discrete_headers "*.h")
# FIXME: exclude headers
install(FILES ${discrete_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/discrete)

# Add all tests
add_subdirectory(tests)
