# The code below allows to clear the PythonLibs cache if we change GTSAM_PYTHON_VERSION
# Inspired from the solution found here: http://blog.bethcodes.com/cmake-tips-tricks-drop-down-list
if((NOT DEFINED GTSAM_LAST_PYTHON_VERSION))
	set(GTSAM_LAST_PYTHON_VERSION ${GTSAM_PYTHON_VERSION} CACHE STRING "Python version used in the last build")
	mark_as_advanced(FORCE GTSAM_LAST_PYTHON_VERSION)
endif()
if((NOT (${GTSAM_PYTHON_VERSION} MATCHES ${GTSAM_LAST_PYTHON_VERSION})))
	unset(PYTHON_INCLUDE_DIR CACHE)
	unset(PYTHON_INCLUDE_DIR2 CACHE)
	unset(PYTHON_LIBRARY CACHE)
	unset(PYTHON_LIBRARY_DEBUG CACHE)
	set(GTSAM_LAST_PYTHON_VERSION ${GTSAM_PYTHON_VERSION} CACHE STRING "Updating python version used in the last build" FORCE)
endif()

# Be sure that python version can be found by FindPythonLibs.cmake
# See: http://stackoverflow.com/a/15660652/2220173
set(Python_ADDITIONAL_VERSIONS ${GTSAM_PYTHON_VERSION})

# Find Python
find_package(PythonLibs ${GTSAM_PYTHON_VERSION} REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

# Boost Python
string(REPLACE "." "" PYTHON_VERSION_NUMBER ${GTSAM_PYTHON_VERSION}) # Remove '.' from version
string(SUBSTRING ${PYTHON_VERSION_NUMBER} 0 2 PYTHON_VERSION_NUMBER) # Trim version number to 2 digits
find_package(Boost COMPONENTS python-py${PYTHON_VERSION_NUMBER} REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

# Numpy_Eigen
include_directories(${CMAKE_SOURCE_DIR}/gtsam/3rdparty/numpy_eigen/include/)

add_subdirectory(handwritten)