# Writes a config file

set(GTSAM_CONFIG_TEMPLATE_PATH ${CMAKE_CURRENT_LIST_DIR})

function(GtsamMakeConfigFile PACKAGE_NAME)

	if(WIN32 AND NOT CYGWIN)
		set(DEF_INSTALL_CMAKE_DIR CMake)
	else()
		set(DEF_INSTALL_CMAKE_DIR lib/cmake/${PACKAGE_NAME})
	endif()

	# Configure extra file
	if(NOT "${ARGV1}" STREQUAL "")
		get_filename_component(name "${ARGV1}" NAME_WE)
		set(EXTRA_FILE "${name}.cmake")
		configure_file(${ARGV1} "${PROJECT_BINARY_DIR}/${EXTRA_FILE}" @ONLY)
		install(FILES "${PROJECT_BINARY_DIR}/${EXTRA_FILE}" DESTINATION "${DEF_INSTALL_CMAKE_DIR}")
	else()
		set(EXTRA_FILE "_does_not_exist_")
	endif()

	# GTSAM defines its version variable as GTSAM_VERSION_STRING while other
	# projects may define it as <ProjectName>_VERSION. Since this file is
	# installed on the system as part of GTSAMCMakeTools and may be useful in
	# external projects, we need to handle both cases of version variable naming
	# here.
	if(NOT DEFINED ${PACKAGE_NAME}_VERSION AND DEFINED ${PACKAGE_NAME}_VERSION_STRING)
		set(${PACKAGE_NAME}_VERSION ${${PACKAGE_NAME}_VERSION_STRING})
	elseif(NOT DEFINED ${PACKAGE_NAME}_VERSION_STRING)
		set(${PACKAGE_NAME}_VERSION ${GTSAM_VERSION_STRING})
	endif()

	# Version file
	include(CMakePackageConfigHelpers)
	write_basic_package_version_file(
	  "${PROJECT_BINARY_DIR}/${PACKAGE_NAME}ConfigVersion.cmake"
	  VERSION ${${PACKAGE_NAME}_VERSION}
	  COMPATIBILITY SameMajorVersion
	)

	# Config file
	file(RELATIVE_PATH CONF_REL_INCLUDE_DIR "${CMAKE_INSTALL_PREFIX}/${DEF_INSTALL_CMAKE_DIR}" "${CMAKE_INSTALL_PREFIX}/include")
	file(RELATIVE_PATH CONF_REL_LIB_DIR "${CMAKE_INSTALL_PREFIX}/${DEF_INSTALL_CMAKE_DIR}" "${CMAKE_INSTALL_PREFIX}/lib")
	configure_file(${GTSAM_CONFIG_TEMPLATE_PATH}/Config.cmake.in "${PROJECT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake" @ONLY)
	message(STATUS "Wrote ${PROJECT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake")

	# Install config, version and exports files (for find scripts)
	install(
		FILES
			"${PROJECT_BINARY_DIR}/${PACKAGE_NAME}Config.cmake"
			"${PROJECT_BINARY_DIR}/${PACKAGE_NAME}ConfigVersion.cmake"
		DESTINATION
			"${CMAKE_INSTALL_PREFIX}/${DEF_INSTALL_CMAKE_DIR}"
	)
	install(EXPORT ${PACKAGE_NAME}-exports DESTINATION ${DEF_INSTALL_CMAKE_DIR})

endfunction()
