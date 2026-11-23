# Authors: Frank Dellaert and Akash Ratheesh

file(GLOB planning_headers "*.h")
install(FILES ${planning_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/planning)

add_subdirectory(tests)
