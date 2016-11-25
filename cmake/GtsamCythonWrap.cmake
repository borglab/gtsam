# Set up cache options
set(GTSAM_CYTHON_INSTALL_PATH "" CACHE PATH "Cython toolbox destination, blank defaults to CMAKE_INSTALL_PREFIX/gtsam_cython")
if(NOT GTSAM_CYTHON_INSTALL_PATH)
	set(GTSAM_CYTHON_INSTALL_PATH "${CMAKE_INSTALL_PREFIX}/gtsam_cython")
endif()

# User-friendly wrapping function.  Builds a mex module from the provided
# interfaceHeader.  For example, for the interface header gtsam.h,
# this will build the wrap module 'gtsam'.
#
# Arguments:
#
# interfaceHeader:  The relative path to the wrapper interface definition file.
# linkLibraries:    Any *additional* libraries to link.  Your project library
#                   (e.g. `lba`), libraries it depends on, and any necessary
#                   MATLAB libraries will be linked automatically.  So normally,
#                   leave this empty.
# extraIncludeDirs: Any *additional* include paths required by dependent
#                   libraries that have not already been added by
#                   include_directories.  Again, normally, leave this empty.
function(wrap_and_install_library_cython interfaceHeader linkLibraries extraIncludeDirs)
	wrap_library_internal_cython("${interfaceHeader}" "${linkLibraries}" "${extraIncludeDirs}")
	install_wrapped_library_internal_cython("${interfaceHeader}")
endfunction()


# Internal function that wraps a library and compiles the wrapper
function(wrap_library_internal_cython interfaceHeader linkLibraries extraIncludeDirs)
	# Wrap codegen interface
    #usage: wrap --cython interfacePath moduleName toolboxPath headerPath
    #  interfacePath : *absolute* path to directory of module interface file
    #  moduleName    : the name of the module, interface file must be called moduleName.h
    #  toolboxPath   : the directory in which to generate the wrappers
    #  headerPath    : path to matlab.h 
	
	# Extract module name from interface header file name
	get_filename_component(interfaceHeader "${interfaceHeader}" ABSOLUTE)
	get_filename_component(modulePath "${interfaceHeader}" PATH)
	get_filename_component(moduleName "${interfaceHeader}" NAME_WE)
	
	# Paths for generated files
	set(generated_files_path "${PROJECT_BINARY_DIR}/cython/${moduleName}")
	set(generated_cpp_file "${generated_files_path}/${moduleName}.cpp")
	
	message(STATUS "Building wrap module ${moduleName}")
	
	# Set up generation of module source file
	file(MAKE_DIRECTORY "${generated_files_path}")
	configure_file(../cython/setup.py.in ${generated_files_path}/setup.py)
	add_custom_command(
		OUTPUT ${generated_cpp_file}
		DEPENDS ${interfaceHeader} wrap ${module_library_target} 
        COMMAND 
            wrap --cython
            ${modulePath}
            ${moduleName} 
            ${generated_files_path} 
			.
			&& python setup.py build_ext --inplace
		VERBATIM
		WORKING_DIRECTORY ${generated_files_path})
		
	# Set up building of mex module
	add_custom_target(${moduleName}_cython_wrapper ALL DEPENDS ${generated_cpp_file} ${interfaceHeader})
    add_custom_target(wrap_${moduleName}_cython_distclean 
	    COMMAND cmake -E remove_directory ${generated_files_path})
endfunction()

# Internal function that installs a wrap toolbox
function(install_wrapped_library_internal_cython interfaceHeader)
	get_filename_component(moduleName "${interfaceHeader}" NAME_WE)
	set(generated_files_path "${PROJECT_BINARY_DIR}/cython/${moduleName}")

    # NOTE: only installs .pxd and .pyx and binary files (not .cpp) - the trailing slash on the directory name
	# here prevents creating the top-level module name directory in the destination.
	message(STATUS "Installing Cython Toolbox to ${GTSAM_CYTHON_INSTALL_PATH}")
	if(GTSAM_BUILD_TYPE_POSTFIXES)
		foreach(build_type ${CMAKE_CONFIGURATION_TYPES})
			string(TOUPPER "${build_type}" build_type_upper)
			if(${build_type_upper} STREQUAL "RELEASE")
				set(build_type_tag "") # Don't create release mode tag on installed directory
			else()
				set(build_type_tag "${build_type}")
			endif()
			# Split up filename to strip trailing '/' in GTSAM_CYTHON_INSTALL_PATH if there is one
			get_filename_component(location "${GTSAM_CYTHON_INSTALL_PATH}" PATH)
			get_filename_component(name "${GTSAM_CYTHON_INSTALL_PATH}" NAME)
			install(DIRECTORY "${generated_files_path}/../" DESTINATION "${location}/${name}${build_type_tag}" 
					CONFIGURATIONS "${build_type}" 
					PATTERN "build" EXCLUDE
					PATTERN "CMakeFiles" EXCLUDE
					PATTERN "Makefile" EXCLUDE
					PATTERN "*.cmake" EXCLUDE
					PATTERN "*.cpp" EXCLUDE
					PATTERN "*.py" EXCLUDE)
		endforeach()
	else()
		install(DIRECTORY "${generated_files_path}/../" DESTINATION ${GTSAM_CYTHON_INSTALL_PATH} 
				PATTERN "build" EXCLUDE
				PATTERN "CMakeFiles" EXCLUDE
				PATTERN "Makefile" EXCLUDE
				PATTERN "*.cmake" EXCLUDE
				PATTERN "*.cpp" EXCLUDE
				PATTERN "*.py" EXCLUDE)
	endif()
endfunction()

# Helper function to install Cython scripts and handle multiple build types where the scripts
# should be installed to all build type toolboxes
function(install_cython_scripts source_directory patterns)
	set(patterns_args "")
	set(exclude_patterns "")

	foreach(pattern ${patterns})
		list(APPEND patterns_args PATTERN "${pattern}")
	endforeach()
	if(GTSAM_BUILD_TYPE_POSTFIXES)
		foreach(build_type ${CMAKE_CONFIGURATION_TYPES})
			string(TOUPPER "${build_type}" build_type_upper)
			if(${build_type_upper} STREQUAL "RELEASE")
				set(build_type_tag "") # Don't create release mode tag on installed directory
			else()
				set(build_type_tag "${build_type}")
			endif()
			# Split up filename to strip trailing '/' in GTSAM_CYTHON_INSTALL_PATH if there is one
			get_filename_component(location "${GTSAM_CYTHON_INSTALL_PATH}" PATH)
			get_filename_component(name "${GTSAM_CYTHON_INSTALL_PATH}" NAME)
			install(DIRECTORY "${source_directory}" DESTINATION "${location}/${name}${build_type_tag}" CONFIGURATIONS "${build_type}"
				    FILES_MATCHING ${patterns_args} PATTERN "${exclude_patterns}" EXCLUDE)
		endforeach()
	else()
		install(DIRECTORY "${source_directory}" DESTINATION "${GTSAM_CYTHON_INSTALL_PATH}" FILES_MATCHING ${patterns_args} PATTERN "${exclude_patterns}" EXCLUDE)
	endif()

endfunction()

