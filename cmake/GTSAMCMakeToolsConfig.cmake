# This config file modifies CMAKE_MODULE_PATH so that the GTSAM-CMakeTools files may be included

set(GTSAM_CMAKE_TOOLS_DIR "${CMAKE_CURRENT_LIST_DIR}")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")
