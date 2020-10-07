if(GTSAM_BUILD_PYTHON)
    if(${GTSAM_PYTHON_VERSION} STREQUAL "Default")
        # Get info about the Python3 interpreter
        # https://cmake.org/cmake/help/latest/module/FindPython3.html#module:FindPython3
        find_package(Python3 COMPONENTS Interpreter Development)

        if(NOT ${Python3_FOUND})
            message(FATAL_ERROR "Cannot find Python3 interpreter. Please install Python >= 3.6.")
        endif()

        set(GTSAM_PYTHON_VERSION "${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}"
                CACHE
                STRING
                "The version of Python to build the wrappers against."
                FORCE)
    endif()

    if(GTSAM_UNSTABLE_BUILD_PYTHON)
        if (NOT GTSAM_BUILD_UNSTABLE)
            message(WARNING "GTSAM_UNSTABLE_BUILD_PYTHON requires the unstable module to be enabled.")
            set(GTSAM_UNSTABLE_BUILD_PYTHON OFF)
        endif()
    endif()

    set(GTSAM_PY_INSTALL_PATH "${CMAKE_INSTALL_PREFIX}/python")
endif()
