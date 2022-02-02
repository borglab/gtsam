# Utilities to help with wrapping.

# Use CMake's find_package to find the version of Python installed.
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

# Depending on the version of CMake, ensure all the appropriate variables are set.
macro(configure_python_variables)
  if(${CMAKE_VERSION} VERSION_LESS "3.12.0")
    set(Python_VERSION_MAJOR
        ${PYTHON_VERSION_MAJOR}
        CACHE INTERNAL "")
    set(Python_VERSION_MINOR
        ${PYTHON_VERSION_MINOR}
        CACHE INTERNAL "")
    set(Python_VERSION_PATCH
        ${PYTHON_VERSION_PATCH}
        CACHE INTERNAL "")
    set(Python_EXECUTABLE
        ${PYTHON_EXECUTABLE}
        CACHE PATH "")

  else()
    # Set both sets of variables
    set(PYTHON_VERSION_MAJOR
        ${Python_VERSION_MAJOR}
        CACHE INTERNAL "")
    set(PYTHON_VERSION_MINOR
        ${Python_VERSION_MINOR}
        CACHE INTERNAL "")
    set(PYTHON_VERSION_PATCH
        ${Python_VERSION_PATCH}
        CACHE INTERNAL "")
    set(PYTHON_EXECUTABLE
        ${Python_EXECUTABLE}
        CACHE PATH "")

  endif()
endmacro()

# Set the Python version for the wrapper and set the paths to the executable and
# include/library directories.
# WRAP_PYTHON_VERSION (optionally) can be "Default" or a
# specific major.minor version.
macro(gtwrap_get_python_version)
  # Unset these cached variables to avoid surprises when the python in the
  # current environment are different from the cached!
  unset(Python_EXECUTABLE CACHE)
  unset(Python_INCLUDE_DIRS CACHE)
  unset(Python_VERSION_MAJOR CACHE)
  unset(Python_VERSION_MINOR CACHE)
  unset(Python_VERSION_PATCH CACHE)

  # Set default value if the parameter is not passed in
  if(NOT WRAP_PYTHON_VERSION)
    set(WRAP_PYTHON_VERSION "Default")
  endif()

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

  # (Always) Configure the variables once we find the python package
  configure_python_variables()

endmacro()

# Concatenate multiple wrapper interface headers into one.
# The concatenation will be (re)performed if and only if any interface files
# change.
#
# Arguments:
# ~~~
# destination: The concatenated master interface header file will be placed here.
# inputs (optional): All the input interface header files
function(combine_interface_headers
         destination
         #inputs
         )
  # check if any interface headers changed
  foreach(INTERFACE_FILE ${ARGN})
    if(NOT EXISTS ${destination} OR
      ${INTERFACE_FILE} IS_NEWER_THAN ${destination})
      set(UPDATE_INTERFACE TRUE)
    endif()
    # trigger cmake on file change
    set_property(DIRECTORY
                 APPEND
                 PROPERTY CMAKE_CONFIGURE_DEPENDS ${INTERFACE_FILE})
  endforeach()
  # if so, then update the overall interface file
  if (UPDATE_INTERFACE)
    file(WRITE ${destination} "")
    # append additional interface headers to end of gtdynamics.i
    foreach(INTERFACE_FILE ${ARGN})
      file(READ ${INTERFACE_FILE} interface_contents)
      file(APPEND ${destination} "${interface_contents}")
    endforeach()
  endif()
endfunction()
