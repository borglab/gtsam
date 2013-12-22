# print configuration variables
# Usage:
#print_config_flag(${GTSAM_BUILD_TESTS} "Build Tests                ")
function(print_config_flag flag msg)
    if (flag)
        message(STATUS "  ${msg}: Enabled")
    else ()
        message(STATUS "  ${msg}: Disabled")
    endif ()
endfunction(print_config_flag)
