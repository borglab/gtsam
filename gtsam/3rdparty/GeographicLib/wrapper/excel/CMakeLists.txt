cmake_minimum_required (VERSION 3.17.0)
project (cgeodesic)

# Set a default build type for single-configuration cmake generators if
# no build type is set.
if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Release)
endif ()

# Make the compiler more picky.
if (MSVC)
  string (REGEX REPLACE "/W[0-4]" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
  string (REGEX REPLACE "/W[0-4]" "" CMAKE_C_FLAGS "${CMAKE_C_FLAGS}")
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /W4")
else ()
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wextra")
  message (WARNING "This has only been tested on Windows")
endif ()

find_package (GeographicLib 1.51 REQUIRED COMPONENTS SHARED)

set (CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
add_library (${PROJECT_NAME} SHARED ${PROJECT_NAME}.cpp)
target_link_libraries (${PROJECT_NAME} ${GeographicLib_LIBRARIES})

if (WIN32)
  if (CMAKE_SIZEOF_VOID_P EQUAL 4)
    set (INSTALL_MSG
      "copy DLLs fron ${CMAKE_CFG_INTDIR} to C:\\Program Files (x86)\\Microsoft Office\\root\\Office16")
  else ()
    set (INSTALL_MSG "copy DLLs from ${CMAKE_CFG_INTDIR} to C:\\Program Files\\Microsoft Office\\root\\Office16")
  endif ()
elseif (APPLE)
  set (INSTALL_MSG "copy DYLIBs from ${CMAKE_CFG_INTDIR} to /Library/Application Support/Microsoft")
endif ()

get_target_property (GEOGRAPHICLIB_LIB_TYPE ${GeographicLib_LIBRARIES} TYPE)
if (GEOGRAPHICLIB_LIB_TYPE STREQUAL "SHARED_LIBRARY")
  if (TRUE)
    add_custom_command (TARGET ${PROJECT_NAME} POST_BUILD
      COMMAND
        ${CMAKE_COMMAND} -E
        copy $<TARGET_FILE:${GeographicLib_LIBRARIES}> ${CMAKE_CFG_INTDIR}
      COMMENT "Installing shared library in build tree
${INSTALL_MSG}")
  else ()
    # Set the run time path for shared libraries for non-Windows machines.
    set_target_properties (${PROJECT_NAME}
      PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
  endif ()
endif ()
