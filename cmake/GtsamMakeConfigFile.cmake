# Writes a config file

function(GtsamMakeConfigFile PACKAGE_NAME)

  if(WIN32 AND NOT CYGWIN)
    set(DEF_INSTALL_CMAKE_DIR CMake)
  else()
    set(DEF_INSTALL_CMAKE_DIR lib/cmake/${PACKAGE_NAME})
  endif()

  file(RELATIVE_PATH CONF_REL_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/${DEF_INSTALL_CMAKE_DIR}" "${CMAKE_INSTALL_PREFIX}/include")
  file(RELATIVE_PATH CONF_REL_LIB_DIR "${CMAKE_INSTALL_PREFIX}/${DEF_INSTALL_CMAKE_DIR}" "${CMAKE_INSTALL_PREFIX}/lib")
  configure_file(${PROJECT_SOURCE_DIR}/cmake/Config.cmake.in "${PROJECT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake" @ONLY)
  message(STATUS "Wrote ${PROJECT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake")

  # Install config and exports files (for find scripts)
  install(FILES "${PROJECT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake" DESTINATION "${CMAKE_INSTALL_PREFIX}/${DEF_INSTALL_CMAKE_DIR}")
  install(EXPORT ${PACKAGE_NAME}-exports DESTINATION ${DEF_INSTALL_CMAKE_DIR})

endfunction()
