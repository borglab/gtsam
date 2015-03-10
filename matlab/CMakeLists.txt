# Install matlab components

include(GtsamMatlabWrap)

# Tests
#message(STATUS "Installing Matlab Toolbox")
install_matlab_scripts("${GTSAM_SOURCE_ROOT_DIR}/matlab/" "*.m;*.fig")
install_matlab_scripts("${GTSAM_SOURCE_ROOT_DIR}/matlab/" "README-gtsam-toolbox.txt")

# Examples
#message(STATUS "Installing Matlab Toolbox Examples")
# Matlab files: *.m and *.fig
#install_matlab_scripts("${GTSAM_SOURCE_ROOT_DIR}/matlab/gtsam_examples" "*.m;*.fig")

# Utilities
#message(STATUS "Installing Matlab Toolbox Utilities")
#install_matlab_scripts("${GTSAM_SOURCE_ROOT_DIR}/matlab/+gtsam" "*.m")

#message(STATUS "Installing Matlab Toolbox Example Data")
# Data files: *.graph, *.mat, and *.txt
file(GLOB matlab_examples_data_graph "${GTSAM_SOURCE_ROOT_DIR}/examples/Data/*.graph")
file(GLOB matlab_examples_data_mat "${GTSAM_SOURCE_ROOT_DIR}/examples/Data/*.mat")
file(GLOB matlab_examples_data_txt "${GTSAM_SOURCE_ROOT_DIR}/examples/Data/*.txt")
set(matlab_examples_data ${matlab_examples_data_graph} ${matlab_examples_data_mat} ${matlab_examples_data_txt}) 
if(GTSAM_BUILD_TYPE_POSTFIXES)
	foreach(build_type ${CMAKE_CONFIGURATION_TYPES})
		string(TOUPPER "${build_type}" build_type_upper)
		if(${build_type_upper} STREQUAL "RELEASE")
			set(build_type_tag "") # Don't create release mode tag on installed directory
		else()
			set(build_type_tag "${build_type}")
		endif()
		# Split up filename to strip trailing '/' in GTSAM_TOOLBOX_INSTALL_PATH if there is one
		get_filename_component(location "${GTSAM_TOOLBOX_INSTALL_PATH}" PATH)
		get_filename_component(name "${GTSAM_TOOLBOX_INSTALL_PATH}" NAME)
		install(FILES ${matlab_examples_data} DESTINATION "${location}/${name}${build_type_tag}/gtsam_examples/Data" CONFIGURATIONS "${build_type}")
	endforeach()
else()
	install(FILES ${matlab_examples_data} DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH}/gtsam_examples/Data)
endif()

