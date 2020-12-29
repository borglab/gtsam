# Check for build of Unstable modules
if(GTSAM_BUILD_PYTHON)
    if(GTSAM_UNSTABLE_BUILD_PYTHON)
        if (NOT GTSAM_BUILD_UNSTABLE)
            message(WARNING "GTSAM_UNSTABLE_BUILD_PYTHON requires the unstable module to be enabled.")
            set(GTSAM_UNSTABLE_BUILD_PYTHON OFF)
        endif()
    endif()

    set(GTSAM_PY_INSTALL_PATH "${CMAKE_INSTALL_PREFIX}/python")
endif()
