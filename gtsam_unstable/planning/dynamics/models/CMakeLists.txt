# Authors: Frank Dellaert and Akash Ratheesh

# Install headers
file(GLOB planning_dynamics_models_headers "*.h")
install(FILES ${planning_dynamics_models_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam_unstable/planning/dynamics/models)
