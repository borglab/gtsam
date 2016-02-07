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

if(GTSAM_PYTHON_VERSION STREQUAL "Default")
	# Search the default version.
	find_package(PythonInterp)
	find_package(PythonLibs)
else()
	find_package(PythonInterp ${GTSAM_PYTHON_VERSION})
	find_package(PythonLibs ${GTSAM_PYTHON_VERSION})
endif()

# Find NumPy C-API -- this is part of the numpy package
find_package(NumPy)

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

# Find Boost Python
find_package(Boost COMPONENTS python${BOOST_PYTHON_VERSION_SUFFIX})

if(Boost_PYTHON${BOOST_PYTHON_VERSION_SUFFIX_UPPERCASE}_FOUND AND PYTHONLIBS_FOUND AND NUMPY_FOUND)
	# Build library
	include_directories(${NUMPY_INCLUDE_DIRS})
	include_directories(${PYTHON_INCLUDE_DIRS})
	include_directories(${Boost_INCLUDE_DIRS})
	include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/)

	# Build the python module library
	add_subdirectory(handwritten)

	# Create and invoke setup.py, see https://bloerg.net/2012/11/10/cmake-and-distutils.html
    set(SETUP_PY_IN "${CMAKE_CURRENT_SOURCE_DIR}/setup.py.in")
    set(SETUP_PY    "${CMAKE_CURRENT_BINARY_DIR}/setup.py")

	# Hacky way to figure out install folder - valid for Linux & Mac
	# default pattern: prefix/lib/pythonX.Y/site-packages from https://docs.python.org/2/install/ 
	SET(PY_INSTALL_FOLDER "${CMAKE_INSTALL_PREFIX}/lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages") 

    configure_file(${SETUP_PY_IN} ${SETUP_PY})

	# TODO(frank): possibly support a different prefix a la matlab wrapper
    install(CODE "execute_process(WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR} COMMAND ${PYTHON_EXECUTABLE} setup.py -v install --prefix ${CMAKE_INSTALL_PREFIX})")
else()
	# Disable python module if we didn't find required libraries
	# message will print at end of main CMakeLists.txt
	SET(GTSAM_PYTHON_WARNINGS "Python dependencies not found - Python module will not be built. Set GTSAM_BUILD_PYTHON to 'Off' to disable this warning. Details:")
	
	if(NOT PYTHONLIBS_FOUND)
		if(GTSAM_PYTHON_VERSION STREQUAL "Default")
			SET(GTSAM_PYTHON_WARNINGS "${GTSAM_PYTHON_WARNINGS}\n -- Default PythonLibs not found")
		else()
			SET(GTSAM_PYTHON_WARNINGS "${GTSAM_PYTHON_WARNINGS}\n -- PythonLibs version ${GTSAM_PYTHON_VERSION} not found")
		endif()
	endif()
	
	if(NOT NUMPY_FOUND)
    	SET(GTSAM_PYTHON_WARNINGS  "${GTSAM_PYTHON_WARNINGS}\n -- Numpy not found")
	endif()
	
	if(NOT Boost_PYTHON${BOOST_PYTHON_VERSION_SUFFIX_UPPERCASE}_FOUND)
		if(GTSAM_PYTHON_VERSION STREQUAL "Default")
			SET(GTSAM_PYTHON_WARNINGS "${GTSAM_PYTHON_WARNINGS}\n -- Default Boost python not found")
		else()
			SET(GTSAM_PYTHON_WARNINGS "${GTSAM_PYTHON_WARNINGS}\n -- Boost Python for python ${GTSAM_PYTHON_VERSION} not found")
		endif()
	endif()
	
	# make available at top-level
	SET(GTSAM_PYTHON_WARNINGS ${GTSAM_PYTHON_WARNINGS} PARENT_SCOPE)
	
endif()
