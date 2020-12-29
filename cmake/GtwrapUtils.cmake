# Utilities to help with wrapping.

function(get_python_version)
  if(${CMAKE_VERSION} VERSION_LESS "3.12.0")
    # Use older version of cmake's find_python
    find_package(PythonInterp)

    if(NOT ${PYTHONINTERP_FOUND})
      message(
        FATAL_ERROR
          "Cannot find Python interpreter. Please install Python >= 3.6.")
    endif()

    find_package(PythonLibs ${PYTHON_VERSION_STRING})

    set(Python_VERSION_MAJOR
        ${PYTHON_VERSION_MAJOR}
        PARENT_SCOPE)
    set(Python_VERSION_MINOR
        ${PYTHON_VERSION_MINOR}
        PARENT_SCOPE)
    set(Python_EXECUTABLE
        ${PYTHON_EXECUTABLE}
        PARENT_SCOPE)

  else()
    # Get info about the Python3 interpreter
    # https://cmake.org/cmake/help/latest/module/FindPython3.html#module:FindPython3
    find_package(Python3 COMPONENTS Interpreter Development)

    if(NOT ${Python3_FOUND})
      message(
        FATAL_ERROR
          "Cannot find Python3 interpreter. Please install Python >= 3.6.")
    endif()

    set(Python_VERSION_MAJOR
        ${Python3_VERSION_MAJOR}
        PARENT_SCOPE)
    set(Python_VERSION_MINOR
        ${Python3_VERSION_MINOR}
        PARENT_SCOPE)

  endif()
endfunction()

# Set the Python version for the wrapper and set the paths to the executable and
# include/library directories. WRAP_PYTHON_VERSION can be "Default" or a
# specific major.minor version.
function(gtwrap_get_python_version WRAP_PYTHON_VERSION)
  # Unset these cached variables to avoid surprises when the python in the
  # current environment are different from the cached!
  unset(Python_EXECUTABLE CACHE)
  unset(Python_INCLUDE_DIRS CACHE)
  unset(Python_VERSION_MAJOR CACHE)
  unset(Python_VERSION_MINOR CACHE)

  # Allow override
  if(${WRAP_PYTHON_VERSION} STREQUAL "Default")
    # Check for Python3 or Python2 in order
    get_python_version()

    set(WRAP_PYTHON_VERSION
        "${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}"
        CACHE STRING "The version of Python to build the wrappers against."
              FORCE)

    # message("========= ${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}")
    # message("========= WRAP_PYTHON_VERSION=${WRAP_PYTHON_VERSION}")
    # message("========= Python_EXECUTABLE=${Python_EXECUTABLE}")

  else()
    find_package(
      Python ${WRAP_PYTHON_VERSION}
      COMPONENTS Interpreter Development
      EXACT REQUIRED)
  endif()

  set(WRAP_PYTHON_VERSION
      ${WRAP_PYTHON_VERSION}
      PARENT_SCOPE)
  set(Python_FOUND
      ${Python_FOUND}
      PARENT_SCOPE)
  set(Python_EXECUTABLE
      ${Python_EXECUTABLE}
      PARENT_SCOPE)
  set(Python_INCLUDE_DIRS
      ${Python_INCLUDE_DIRS}
      PARENT_SCOPE)
  set(Python_LIBRARY_DIRS
      ${Python_LIBRARY_DIRS}
      PARENT_SCOPE)

endfunction()
