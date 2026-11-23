# Authors: Frank Dellaert and Akash Ratheesh

# Install headers
file(GLOB planning_controller_headers "*.h")
install(FILES ${planning_controller_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam_unstable/planning/controller)
