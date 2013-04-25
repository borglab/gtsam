# install CCOLAMD headers
install(FILES CCOLAMD/Include/ccolamd.h DESTINATION include/gtsam/3rdparty/CCOLAMD)
install(FILES UFconfig/UFconfig.h DESTINATION include/gtsam/3rdparty/UFconfig)

# Option for using system Eigen or GTSAM-bundled Eigen
option(GTSAM_USE_SYSTEM_EIGEN "Find and use system-installed Eigen. If 'off', use the one bundled with GTSAM" OFF)

# Switch for using system Eigen or GTSAM-bundled Eigen
if(GTSAM_USE_SYSTEM_EIGEN)
	# Use generic Eigen include paths e.g. <Eigen/Core>
	set(GTSAM_EIGEN_INCLUDE_PREFIX "")
	
	find_package(Eigen3 REQUIRED)
	include_directories(EIGEN3_INCLUDE_DIR)
else()
	# Use bundled Eigen include paths e.g. <gtsam/3rdparty/Eigen/Eigen/Core>
	set(GTSAM_EIGEN_INCLUDE_PREFIX "gtsam/3rdparty/Eigen/")
	
	# Clear any variables set by FindEigen3
	if(EIGEN3_INCLUDE_DIR)
		set(EIGEN3_INCLUDE_DIR NOTFOUND)
	endif()

	# install Eigen - only the headers in our 3rdparty directory
	install(DIRECTORY Eigen/Eigen 
		DESTINATION include/gtsam/3rdparty/Eigen
		FILES_MATCHING PATTERN "*.h")
	file(GLOB eigen_dir_headers_all "Eigen/Eigen/*")

	# ensure that Eigen folders without extensions get added
	foreach(eigen_dir ${eigen_dir_headers_all})
		get_filename_component(filename ${eigen_dir} NAME)
		if (NOT ((${filename} MATCHES "CMakeLists.txt") OR (${filename} MATCHES "src") OR (${filename} MATCHES ".svn")))
			install(FILES Eigen/Eigen/${filename} DESTINATION include/gtsam/3rdparty/Eigen/Eigen)     
		endif()
	endforeach(eigen_dir)
endif()

# Write Eigen include file with the paths for either the system Eigen or the GTSAM-bundled Eigen
configure_file(gtsam_eigen_includes.h.in gtsam_eigen_includes.h)
