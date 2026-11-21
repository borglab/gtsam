# Install headers
file(GLOB mpc_stag_dynamics_models_headers "*.h")
install(FILES ${mpc_stag_dynamics_models_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam_unstable/mpc_stag/dynamics/models)
