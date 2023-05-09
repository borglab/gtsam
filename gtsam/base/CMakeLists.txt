# Install headers
file(GLOB base_headers "*.h")
install(FILES ${base_headers} DESTINATION include/gtsam/base)

file(GLOB base_headers_tree "treeTraversal/*.h")
install(FILES ${base_headers_tree} DESTINATION include/gtsam/base/treeTraversal)

file(GLOB deprecated_headers "deprecated/*.h")
install(FILES ${deprecated_headers} DESTINATION include/gtsam/base/deprecated)

# Build tests
add_subdirectory(tests)
