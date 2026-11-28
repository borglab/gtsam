# Authors: Frank Dellaert and Akash Ratheesh

# Install headers
file(GLOB planning_headers "*.h")
install(FILES ${planning_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam_unstable/planning)

add_subdirectory(dynamics)

# Add all tests
add_subdirectory(tests)
