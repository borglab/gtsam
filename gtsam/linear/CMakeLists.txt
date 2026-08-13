# Install headers
file(GLOB linear_headers "*.h")
install(FILES ${linear_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/linear)
file(GLOB linear_internal_headers "internal/*.h")
install(FILES ${linear_internal_headers}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/linear/internal)

# Build tests
add_subdirectory(tests)
