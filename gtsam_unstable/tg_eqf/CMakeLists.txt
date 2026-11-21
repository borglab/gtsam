# Install headers
file(GLOB tg_eqf_headers "*.h")
install(FILES ${tg_eqf_headers}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam_unstable/tg_eqf)

add_subdirectory(examples)
add_subdirectory(tests)
