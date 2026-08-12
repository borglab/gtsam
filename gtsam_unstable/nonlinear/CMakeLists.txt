# Install headers
file(GLOB nonlinear_headers "*.h")
install(FILES ${nonlinear_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam_unstable/nonlinear)

# Add all tests
add_subdirectory(tests)
