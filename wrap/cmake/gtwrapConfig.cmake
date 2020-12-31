# This config file modifies CMAKE_MODULE_PATH so that the wrap cmake files may
# be included This file also allows the use of `find_package(gtwrap)` in CMake.

set(GTWRAP_DIR "${CMAKE_CURRENT_LIST_DIR}")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")

if(WIN32 AND NOT CYGWIN)
  set(SCRIPT_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/CMake")
else()
  set(SCRIPT_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/lib/cmake")
endif()

include(${SCRIPT_INSTALL_DIR}/gtwrap/PybindWrap.cmake)
include(${SCRIPT_INSTALL_DIR}/gtwrap/GtwrapUtils.cmake)
