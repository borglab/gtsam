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

# David Rosen's Optimization library (header-only; provides INTERFACE target "Optimization")
add_subdirectory(Optimization)

# David Rosen's Preconditioners library (provides target "ILDL", depends on SPQR and OpenMP)
add_subdirectory(Preconditioners)

# If LOBPCG verification is enabled, the gtsam target links Optimization & ILDL,
# so they must be part of the GTSAM-exports install set. The upstream CMakeLists
# use plain source paths in target_include_directories(... PUBLIC ...), which
# install(EXPORT) rejects — rewrite those properties with BUILD_INTERFACE
# wrappers so the include paths only apply at build time.
if(GTSAM_USE_LOBPCG_VERIFICATION)
  get_target_property(_ildl_includes ILDL INTERFACE_INCLUDE_DIRECTORIES)
  set(_ildl_build_includes "")
  foreach(_d ${_ildl_includes})
    # Drop system Eigen — GTSAM ships its own 3rdparty Eigen and must use it
    # consistently. ILDL's Eigen includes are header-only and satisfied by
    # whichever Eigen ends up on the include path.
    if(_d MATCHES "eigen3?$" OR _d MATCHES "/eigen3?/?$")
      continue()
    endif()
    list(APPEND _ildl_build_includes "$<BUILD_INTERFACE:${_d}>")
  endforeach()
  set_target_properties(ILDL PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${_ildl_build_includes}"
    # Treat remaining ILDL paths as SYSTEM so any benign warnings inside them
    # don't trigger gtsam's -Werror.
    INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_ildl_build_includes}")

  get_target_property(_opt_includes Optimization INTERFACE_INCLUDE_DIRECTORIES)
  set(_opt_build_includes "")
  foreach(_d ${_opt_includes})
    list(APPEND _opt_build_includes "$<BUILD_INTERFACE:${_d}>")
  endforeach()
  set_target_properties(Optimization PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${_opt_build_includes}")
  # Clear INTERFACE_SOURCES — Optimization upstream attaches headers there for
  # IDE display; those paths are in the source tree and aren't installable.
  set_target_properties(Optimization PROPERTIES INTERFACE_SOURCES "")

  install(TARGETS Optimization ILDL
    EXPORT GTSAM-exports
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR})
endif()

set(GTSAM_EXPORTED_TARGETS "${GTSAM_EXPORTED_TARGETS}" PARENT_SCOPE)