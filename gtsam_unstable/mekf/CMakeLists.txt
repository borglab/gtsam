# Install headers
file(GLOB mekf_headers "*.h")
install(FILES ${mekf_headers}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam_unstable/mekf)

add_subdirectory(tests)
add_subdirectory(examples)
