# install CCOLAMD and SuiteSparse_config headers
install(FILES CCOLAMD/Include/ccolamd.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/3rdparty/CCOLAMD)
install(FILES SuiteSparse_config/SuiteSparse_config.h DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/3rdparty/SuiteSparse_config)

# install Eigen unless System Eigen is used
if(NOT GTSAM_USE_SYSTEM_EIGEN)
	# Find plain .h files
	file(GLOB_RECURSE eigen_headers "${CMAKE_CURRENT_SOURCE_DIR}/Eigen/Eigen/*.h")

	# Find header files without extension
	file(GLOB eigen_dir_headers_all "Eigen/Eigen/*")
	foreach(eigen_dir ${eigen_dir_headers_all})
		get_filename_component(filename ${eigen_dir} NAME)
		if (NOT ((${filename} MATCHES "CMakeLists.txt") OR (${filename} MATCHES "src") OR (${filename} MATCHES ".svn")))
			list(APPEND eigen_headers "${CMAKE_CURRENT_SOURCE_DIR}/Eigen/Eigen/${filename}")
			install(FILES Eigen/Eigen/${filename} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/3rdparty/Eigen/Eigen)
		endif()
	endforeach(eigen_dir)

	if(GTSAM_WITH_EIGEN_UNSUPPORTED)
		message(STATUS "Installing Eigen Unsupported modules")
		# do the same for the unsupported eigen folder
		file(GLOB_RECURSE  unsupported_eigen_headers "${CMAKE_CURRENT_SOURCE_DIR}/Eigen/unsupported/Eigen/*.h")

		file(GLOB unsupported_eigen_dir_headers_all "Eigen/unsupported/Eigen/*")
		foreach(unsupported_eigen_dir ${unsupported_eigen_dir_headers_all})
			get_filename_component(filename ${unsupported_eigen_dir} NAME)
			if (NOT ((${filename} MATCHES "CMakeLists.txt") OR (${filename} MATCHES "src") OR (${filename} MATCHES "CXX11") OR (${filename} MATCHES ".svn")))
				list(APPEND unsupported_eigen_headers "${CMAKE_CURRENT_SOURCE_DIR}/Eigen/unsupported/Eigen/${filename}")
				install(FILES Eigen/unsupported/Eigen/${filename} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/3rdparty/Eigen/unsupported/Eigen)
			endif()
		endforeach(unsupported_eigen_dir)
	endif()

	# Add to project source
	set(eigen_headers ${eigen_headers} PARENT_SCOPE)
	# set(unsupported_eigen_headers ${unsupported_eigen_headers} PARENT_SCOPE)

	# install Eigen - only the headers in our 3rdparty directory
	install(DIRECTORY Eigen/Eigen
		DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/3rdparty/Eigen
		FILES_MATCHING PATTERN "*.h")

	if(GTSAM_WITH_EIGEN_UNSUPPORTED)
		install(DIRECTORY Eigen/unsupported/Eigen
			DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gtsam/3rdparty/Eigen/unsupported/
			FILES_MATCHING PATTERN "*.h")
	endif()

endif()

# metis: already handled in ROOT/cmake/HandleMetis.cmake

add_subdirectory(ceres)

# Developer Note for GeographicLib updates:
# When updating the bundled GeographicLib, its CMakeLists.txt might need manual
# adjustments to disable building its examples and tools, as they are not
# typically needed or desired when GeographicLib is used as a GTSAM dependency.

# --- GeographicLib Handling ---
# Clear any cached GeographicLib find results to ensure a fresh detection.
unset(GeographicLib_FOUND CACHE)
unset(GeographicLib_INCLUDE_DIRS CACHE)
unset(GeographicLib_LIBRARIES CACHE)
unset(GeographicLib_LIBRARY_DIRS CACHE)
# Use GeographicLib's own find script to locate an existing system installation.
include(GeographicLib/cmake/FindGeographicLib.cmake)

# Determine default for GTSAM_INSTALL_GEOGRAPHICLIB option.
# If system GeographicLib is not found, default to installing the bundled version.
if(NOT GeographicLib_FOUND)
  set(install_geographiclib_default ON)
else()
  set(install_geographiclib_default OFF)
endif()
# Option for users to force build/install of the bundled GeographicLib.
option(GTSAM_INSTALL_GEOGRAPHICLIB "Build and install the 3rd-party library GeographicLib" ${install_geographiclib_default})

# Warn if a system GeographicLib is found but user also opts to install the bundled version.
if(GeographicLib_FOUND AND GTSAM_INSTALL_GEOGRAPHICLIB)
  message(WARNING "GeographicLib is installed on your system and GTSAM_INSTALL_GEOGRAPHICLIB is enabled.  Installing gtsam will either overwrite the installed GeographicLib or install a second version that may conflict.  You may want to disable GTSAM_INSTALL_GEOGRAPHICLIB if the installed version was not installed by GTSAM.")
endif()

# If installing the bundled GeographicLib:
if(GTSAM_INSTALL_GEOGRAPHICLIB)
  # Override GeographicLib's default 'INCDIR' ("include") to use CMAKE_INSTALL_INCLUDEDIR.
  # This ensures its headers are installed to the correct system include path (e.g., for multi-arch),
  # resolving to ${CMAKE_INSTALL_INCLUDEDIR}/GeographicLib.
  set(INCDIR "${CMAKE_INSTALL_INCLUDEDIR}" CACHE PATH "Override GeographicLib's INCDIR for GTSAM bundle" FORCE)
  
  # Disable installation of GeographicLib examples.
  # GeographicLib's CMakeLists.txt checks 'if(EXAMPLEDIR)' before adding its 'examples'
  # subdirectory. Setting EXAMPLEDIR to "OFF" (as a CACHE STRING, matching its original
  # definition type) makes this condition false.
  set(EXAMPLEDIR "OFF" CACHE STRING "Disable GeographicLib example installation for GTSAM bundle" FORCE)

  add_subdirectory(GeographicLib)
endif()

# Propagate any exported targets (like GeographicLib if built) to the parent scope.
