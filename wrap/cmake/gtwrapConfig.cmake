# This config file modifies CMAKE_MODULE_PATH so that the wrap cmake files may be included
# This file also allows the use of `find_package(gtwrap)` in CMake.

set(GTWRAP_DIR "${CMAKE_CURRENT_LIST_DIR}")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")
