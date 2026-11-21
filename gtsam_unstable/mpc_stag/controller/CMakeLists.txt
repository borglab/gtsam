# Install headers
file(GLOB mpc_stag_controller_headers "*.h")
install(FILES ${mpc_stag_controller_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam_unstable/mpc_stag/controller)

add_subdirectory(sampler)
