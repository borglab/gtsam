# Install headers
file(GLOB base_headers "*.h")
install(FILES ${base_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/base)

file(GLOB base_headers_tree "treeTraversal/*.h")
install(FILES ${base_headers_tree} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/base/treeTraversal)

file(GLOB base_cuda_headers "cuda/*.h")
install(FILES ${base_cuda_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/base/cuda)

# Build tests
add_subdirectory(tests)
