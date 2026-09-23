###############################################################################
# Support ccache, if installed
if(NOT MSVC AND NOT XCODE_VERSION)
    find_program(CCACHE_FOUND ccache)
    if(CCACHE_FOUND)
        if(GTSAM_BUILD_WITH_CCACHE)
            # Compiles only. ccache cannot cache a link, so launching the
            # linker through it just adds a process per link and counts them
            # as uncacheable calls, which buries the real hit rate.
            set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
        else()
            set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "")
            set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK "")
        endif()
    endif(CCACHE_FOUND)
endif()
