option(GTSAM_USE_SYSTEM_NUMPY_C_API "Find and use system-installed NumPy C-API. If 'off', use the one bundled with GTSAM" OFF)

# Guard to avoid breaking this code in ccmake if by accident GTSAM_PYTHON_VERSION is set to an empty string 
if(GTSAM_PYTHON_VERSION STREQUAL "")
	set(GTSAM_PYTHON_VERSION "Default" CACHE STRING "Target python version for GTSAM python module. Use 'Default' to chose the default version" FORCE)
endif()

# The code below allows to clear the PythonLibs cache if we change GTSAM_PYTHON_VERSION
# Inspired from the solution found here: http://blog.bethcodes.com/cmake-tips-tricks-drop-down-list
if(NOT DEFINED GTSAM_LAST_PYTHON_VERSION)
	set(GTSAM_LAST_PYTHON_VERSION ${GTSAM_PYTHON_VERSION} CACHE STRING "Python version used in the last build")
	mark_as_advanced(FORCE GTSAM_LAST_PYTHON_VERSION)
endif()
if(NOT (${GTSAM_PYTHON_VERSION} MATCHES ${GTSAM_LAST_PYTHON_VERSION}))
	unset(PYTHON_INCLUDE_DIR CACHE)
	unset(PYTHON_INCLUDE_DIR2 CACHE)
	unset(PYTHON_LIBRARY CACHE)
	unset(PYTHON_LIBRARY_DEBUG CACHE)
	set(GTSAM_LAST_PYTHON_VERSION ${GTSAM_PYTHON_VERSION} CACHE STRING "Updating python version used in the last build" FORCE)
endif()

# Compose strings used to specify the boost python version. They will be empty if we want to use the defaut
if(NOT GTSAM_PYTHON_VERSION STREQUAL "Default")
	string(REPLACE "." "" BOOST_PYTHON_VERSION_SUFFIX ${GTSAM_PYTHON_VERSION})           # Remove '.' from version
	string(SUBSTRING ${BOOST_PYTHON_VERSION_SUFFIX} 0 2 BOOST_PYTHON_VERSION_SUFFIX)     # Trim version number to 2 digits
	set(BOOST_PYTHON_VERSION_SUFFIX "-py${BOOST_PYTHON_VERSION_SUFFIX}")                 # Append '-py' prefix
	string(TOUPPER ${BOOST_PYTHON_VERSION_SUFFIX} BOOST_PYTHON_VERSION_SUFFIX_UPPERCASE) # Get uppercase string
else()
	set(BOOST_PYTHON_VERSION_SUFFIX "")
	set(BOOST_PYTHON_VERSION_SUFFIX_UPPERCASE "")
endif()

# Find NumPy C-API
if(GTSAM_USE_SYSTEM_NUMPY_C_API)
	find_package(NumPy)
	include_directories(${NUMPY_INCLUDE_DIRS})
else()
	include_directories(${CMAKE_SOURCE_DIR}/gtsam/3rdparty/numpy_c_api/include/)
endif()

# Find Python
# First, be sure that python version can be found by FindPythonLibs.cmake
# See: http://stackoverflow.com/a/15660652/2220173
set(Python_ADDITIONAL_VERSIONS ${GTSAM_PYTHON_VERSION})
# Then look for the the lib. If no version is specified when looking for PythonLibs it searches the default version.
# See: https://cmake.org/cmake/help/v3.1/module/FindPythonInterp.html
if(GTSAM_PYTHON_VERSION STREQUAL "Default")
	find_package(PythonLibs)
else()
	find_package(PythonLibs ${GTSAM_PYTHON_VERSION})
endif()

# Find Boost Python
find_package(Boost COMPONENTS python${BOOST_PYTHON_VERSION_SUFFIX})

# Add handwritten directory if we found python and boost python
if(Boost_PYTHON${BOOST_PYTHON_VERSION_SUFFIX_UPPERCASE}_FOUND AND PYTHONLIBS_FOUND)
	include_directories(${PYTHON_INCLUDE_DIRS})
	include_directories(${Boost_INCLUDE_DIRS})
	include_directories(${CMAKE_SOURCE_DIR}/gtsam/3rdparty/numpy_eigen/include/)

	file(COPY "gtsam" DESTINATION ".")
	file(COPY "setup.py" DESTINATION ".")
	add_subdirectory(handwritten)
# Disable python module if we didn't find required lybraries
else()
	set(GTSAM_BUILD_PYTHON OFF CACHE BOOL "Build Python wrapper statically (increases build time)" FORCE)
endif()

# Print warnings (useful for ccmake)
if(NOT PYTHONLIBS_FOUND)
	if(GTSAM_PYTHON_VERSION STREQUAL "Default")
		message(WARNING "Default PythonLibs was not found -- Python module cannot be built. Option GTSAM_BUILD_PYTHON disabled.")
	else()
		message(WARNING "PythonLibs version ${GTSAM_PYTHON_VERSION} was not found -- Python module cannot be built. Option GTSAM_BUILD_PYTHON disabled.")
	endif()
endif()

if(NOT Boost_PYTHON${BOOST_PYTHON_VERSION_SUFFIX_UPPERCASE}_FOUND)
	if(GTSAM_PYTHON_VERSION STREQUAL "Default")
		message(WARNING "Default Boost python was not found -- Python module cannot be built. Option GTSAM_BUILD_PYTHON disabled.")
	else()
		message(WARNING "Boost Python for python ${GTSAM_PYTHON_VERSION} was not found -- Python module cannot be built. Option GTSAM_BUILD_PYTHON disabled.")
	endif()
endif()
