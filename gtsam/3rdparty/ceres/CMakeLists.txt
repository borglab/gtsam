file(GLOB ceres_headers "${CMAKE_CURRENT_SOURCE_DIR}/*.h")
install(FILES ${ceres_headers} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/3rdparty/ceres)
