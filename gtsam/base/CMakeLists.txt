# Install headers
file(GLOB base_headers "*.h")
install(FILES ${base_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/base)

file(GLOB base_headers_tree "treeTraversal/*.h")
install(FILES ${base_headers_tree} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/base/treeTraversal)

if(GTSAM_ENABLE_CUDA)
  file(GLOB base_cuda_headers "cuda/*.h")
  install(FILES ${base_cuda_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/base/cuda)
endif()

# Build tests
add_subdirectory(tests)
