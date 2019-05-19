project (PyGeographicLib)

cmake_minimum_required (VERSION 2.8.8)

# Set a default build type for single-configuration cmake generators if
# no build type is set.
if (NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE Release)
endif ()

# Make the compiler more picky.
if (MSVC)
  string (REGEX REPLACE "/W[0-4]" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
else ()
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
endif ()

# The version of python that boost-python uses.  Also used for the
# installation directory.
set (PYTHON_VERSION 2.7)

# Requires python + python devel
find_package (PythonLibs ${PYTHON_VERSION} REQUIRED)

# Required boost-python + boost-devel
find_package (Boost REQUIRED COMPONENTS python)

find_package (GeographicLib REQUIRED COMPONENTS SHARED)

include_directories (${GeographicLib_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS})

add_library (${PROJECT_NAME} MODULE ${PROJECT_NAME}.cpp)

get_target_property (GEOGRAPHICLIB_LIB_TYPE ${GeographicLib_LIBRARIES} TYPE)
if (GEOGRAPHICLIB_LIB_TYPE STREQUAL "SHARED_LIBRARY")
  if (WIN32)
    add_custom_command (TARGET ${PROJECT_NAME} POST_BUILD
      COMMAND
        ${CMAKE_COMMAND} -E
        copy $<TARGET_FILE:${GeographicLib_LIBRARIES}> ${CMAKE_CFG_INTDIR}
      COMMENT "Installing shared library in build tree")
  else ()
    # Set the run time path for shared libraries for non-Windows machines.
    set_target_properties (${PROJECT_NAME}
      PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
  endif ()
endif ()

# Don't include the "lib" prefix on the output name
set_target_properties (${PROJECT_NAME} PROPERTIES PREFIX "")
target_link_libraries (${PROJECT_NAME} ${GeographicLib_LIBRARIES}
  ${Boost_LIBRARIES} ${PYTHON_LIBRARIES})

install (TARGETS ${PROJECT_NAME} LIBRARY
  # if CMAKE_INSTALL_PREFIX=~/.local then this specifies a directory in
  # the default path.
  DESTINATION lib/python${PYTHON_VERSION}/site-packages)
