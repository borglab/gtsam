# Install headers
file(GLOB basis_headers "*.h")
install(FILES ${basis_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/basis)

# Build tests
add_subdirectory(tests)
