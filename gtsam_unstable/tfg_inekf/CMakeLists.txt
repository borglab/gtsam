# Install headers
file(GLOB tfg_inekf_headers "*.h")
install(FILES ${tfg_inekf_headers}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam_unstable/tfg_inekf)

add_subdirectory(tests)
add_subdirectory(examples)
