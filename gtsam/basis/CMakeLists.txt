# Install headers
file(GLOB basis_headers "*.h")
install(FILES ${basis_headers} DESTINATION include/gtsam/basis)

# Build tests
add_subdirectory(tests)
