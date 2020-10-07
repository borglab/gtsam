###############################################################################
# Support ccache, if installed
if(NOT MSVC AND NOT XCODE_VERSION)
    find_program(CCACHE_FOUND ccache)
    if(CCACHE_FOUND)
        if(GTSAM_BUILD_WITH_CCACHE)
            set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
            set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
        else()
            set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "")
            set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "")
        endif()
    endif(CCACHE_FOUND)
endif()
