# Install headers
file(GLOB mpc_stag_headers "*.h")
install(FILES ${mpc_stag_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam_unstable/mpc_stag)

add_subdirectory(cost)
add_subdirectory(dynamics)

# Add all tests
add_subdirectory(tests)
