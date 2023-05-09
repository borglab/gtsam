# Set Python version if either Python or MATLAB wrapper is requested.
if(GTSAM_BUILD_PYTHON OR GTSAM_INSTALL_MATLAB_TOOLBOX)
  if(${GTSAM_PYTHON_VERSION} STREQUAL "Default")

    if(${CMAKE_VERSION} VERSION_LESS "3.12.0")
      # Use older version of cmake's find_python
      find_package(PythonInterp)

      if(NOT ${PYTHONINTERP_FOUND})
        message(
          FATAL_ERROR
            "Cannot find Python interpreter. Please install Python >= 3.6.")
      endif()

      find_package(PythonLibs ${PYTHON_VERSION_STRING})

      set(Python_VERSION_MAJOR ${PYTHON_VERSION_MAJOR})
      set(Python_VERSION_MINOR ${PYTHON_VERSION_MINOR})
      set(Python_EXECUTABLE ${PYTHON_EXECUTABLE})

    else()
      # Get info about the Python3 interpreter
      # https://cmake.org/cmake/help/latest/module/FindPython3.html#module:FindPython3
      find_package(Python3 COMPONENTS Interpreter Development)

      if(NOT ${Python3_FOUND})
        message(
          FATAL_ERROR
            "Cannot find Python3 interpreter. Please install Python >= 3.6.")
      endif()

      set(Python_VERSION_MAJOR ${Python3_VERSION_MAJOR})
      set(Python_VERSION_MINOR ${Python3_VERSION_MINOR})

    endif()

    set(GTSAM_PYTHON_VERSION
        "${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}"
        CACHE STRING "The version of Python to build the wrappers against."
              FORCE)

  endif()
endif()

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
