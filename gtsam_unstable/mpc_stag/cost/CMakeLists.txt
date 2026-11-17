# Install headers
file(GLOB mpc_stag_cost_headers "*.h")
install(FILES ${mpc_stag_cost_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam_unstable/mpc_stag/cost)
