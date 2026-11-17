# Install headers
file(GLOB mpc_stag_controller_sampler_headers "*.h")
install(FILES ${mpc_stag_controller_sampler_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam_unstable/mpc_stag/controller/sampler)
