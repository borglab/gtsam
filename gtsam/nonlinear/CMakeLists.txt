# Install headers
file(GLOB nonlinear_headers "*.h")
install(FILES ${nonlinear_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/nonlinear)

file(GLOB nonlinear_headers_internal "internal/*.h")
install(FILES ${nonlinear_headers_internal} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/nonlinear/internal)

file(GLOB nonlinear_cuda_headers "cuda/*.h")
install(FILES ${nonlinear_cuda_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/nonlinear/cuda)

# Build tests
add_subdirectory(tests)
