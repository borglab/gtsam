# Install headers
set(subdir hybrid)
file(GLOB hybrid_headers "*.h")
install(FILES ${hybrid_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/hybrid)

file(GLOB hybrid_internal_headers "internal/*.h")
install(FILES ${hybrid_internal_headers}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/hybrid/internal)

# Add all tests
add_subdirectory(tests)
