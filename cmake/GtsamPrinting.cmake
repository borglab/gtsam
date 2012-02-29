# print configuration variables
# Usage:
#print_config_flag(${GTSAM_BUILD_TESTS} "Build Tests                ")
macro(print_config_flag flag msg)
    if ("${flag}" STREQUAL "ON")
        message(STATUS "  ${msg}: Enabled")
    else ("${flag}" STREQUAL "ON")
        message(STATUS "  ${msg}: Disabled")
    endif ("${flag}" STREQUAL "ON")
endmacro(print_config_flag)