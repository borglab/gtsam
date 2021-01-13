# Utilities to help with wrapping.

macro(get_python_version)
  if(${CMAKE_VERSION} VERSION_LESS "3.12.0")
    # Use older version of cmake's find_python
    find_package(PythonInterp)

    if(NOT ${PYTHONINTERP_FOUND})
      message(
        FATAL_ERROR
          "Cannot find Python interpreter. Please install Python>=3.5.")
    endif()

    find_package(PythonLibs ${PYTHON_VERSION_STRING})

    set(Python_VERSION_MAJOR
        ${PYTHON_VERSION_MAJOR}
        PARENT_SCOPE)
    set(Python_VERSION_MINOR
        ${PYTHON_VERSION_MINOR}
        PARENT_SCOPE)
    set(Python_VERSION_PATCH
        ${PYTHON_VERSION_PATCH}
        PARENT_SCOPE)
    set(Python_EXECUTABLE
        ${PYTHON_EXECUTABLE}
        PARENT_SCOPE)

  else()
    # Get info about the Python interpreter
    # https://cmake.org/cmake/help/latest/module/FindPython.html#module:FindPython
    find_package(Python COMPONENTS Interpreter Development)

    if(NOT ${Python_FOUND})
      message(
        FATAL_ERROR
          "Cannot find Python interpreter. Please install Python>=3.5.")
    endif()

  endif()
endmacro()

# Set the Python version for the wrapper and set the paths to the executable and
# include/library directories. WRAP_PYTHON_VERSION can be "Default" or a
# specific major.minor version.
macro(gtwrap_get_python_version WRAP_PYTHON_VERSION)
  # Unset these cached variables to avoid surprises when the python in the
  # current environment are different from the cached!
  unset(Python_EXECUTABLE CACHE)
  unset(Python_INCLUDE_DIRS CACHE)
  unset(Python_VERSION_MAJOR CACHE)
  unset(Python_VERSION_MINOR CACHE)
  unset(Python_VERSION_PATCH CACHE)

  # Allow override
  if(${WRAP_PYTHON_VERSION} STREQUAL "Default")
    # Check for Python3 or Python2 in order
    get_python_version()

    # Set the wrapper python version
    set(WRAP_PYTHON_VERSION
        "${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}.${Python_VERSION_PATCH}"
        CACHE STRING "The version of Python to build the wrappers against."
              FORCE)

  else()
    # Find the Python that best matches the python version specified.
    find_package(
      Python ${WRAP_PYTHON_VERSION}
      COMPONENTS Interpreter Development
      EXACT)
  endif()

endmacro()
