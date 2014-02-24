# install CCOLAMD headers
install(FILES CCOLAMD/Include/ccolamd.h DESTINATION include/gtsam/3rdparty/CCOLAMD)
install(FILES UFconfig/UFconfig.h DESTINATION include/gtsam/3rdparty/UFconfig)

if(NOT GTSAM_USE_SYSTEM_EIGEN)
	# Find plain .h files
	file(GLOB_RECURSE eigen_headers "${CMAKE_CURRENT_SOURCE_DIR}/Eigen/Eigen/*.h")

	# Find header files without extension
	file(GLOB eigen_dir_headers_all "Eigen/Eigen/*")
	foreach(eigen_dir ${eigen_dir_headers_all})
		get_filename_component(filename ${eigen_dir} NAME)
		if (NOT ((${filename} MATCHES "CMakeLists.txt") OR (${filename} MATCHES "src") OR (${filename} MATCHES ".svn")))
			list(APPEND eigen_headers "${CMAKE_CURRENT_SOURCE_DIR}/Eigen/Eigen/${filename}")
			install(FILES Eigen/Eigen/${filename} DESTINATION include/gtsam/3rdparty/Eigen/Eigen)
		endif()
	endforeach(eigen_dir)
	
	# Add to project source
	set(eigen_headers ${eigen_headers} PARENT_SCOPE)
	
	# install Eigen - only the headers in our 3rdparty directory
	install(DIRECTORY Eigen/Eigen 
		DESTINATION include/gtsam/3rdparty/Eigen
		FILES_MATCHING PATTERN "*.h")
endif()

############ NOTE:  When updating GeographicLib be sure to disable building their examples
############        and unit tests by commenting out their lines:
# add_subdirectory (examples)
# set (TOOLS CartConvert ConicProj GeodesicProj GeoConvert GeodSolve
#   GeoidEval Gravity MagneticField Planimeter TransverseMercatorProj)
# add_subdirectory (tools)

add_subdirectory(GeographicLib)
