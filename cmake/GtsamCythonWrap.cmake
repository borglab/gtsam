# Check Cython version, need to be >=0.25.2
# Unset these cached variables to avoid surprises when the python/cython
# in the current environment are different from the cached!
unset(PYTHON_EXECUTABLE CACHE)
unset(CYTHON_EXECUTABLE CACHE)
find_package(Cython 0.25.2 REQUIRED)

# Set up cache options
set(GTSAM_CYTHON_INSTALL_PATH "" CACHE PATH "Cython toolbox destination, blank defaults to CMAKE_INSTALL_PREFIX/gtsam_cython")
if(NOT GTSAM_CYTHON_INSTALL_PATH)
	set(GTSAM_CYTHON_INSTALL_PATH "${CMAKE_INSTALL_PREFIX}/cython")
endif()

# User-friendly Cython wrapping and installing function.  
# Builds a Cython module from the provided interface_header.  
# For example, for the interface header gtsam.h,
# this will build the wrap module 'gtsam'.
#
# Arguments:
#
# interface_header:  The relative path to the wrapper interface definition file.
# extra_imports: extra header to import in the Cython pxd file. 
#                For example, to use Cython gtsam.pxd in your own module, 
#				 use "from gtsam cimport *"
# setup_py_in_path: Path to the setup.py.in config file, which will be converted
#                   to setup.py file by cmake and used to compile the Cython module
#                   by invoking "python setup.py build_ext --inplace"
# install_path: destination to install the library
function(wrap_and_install_library_cython interface_header extra_imports setup_py_in_path install_path)
	# Paths for generated files
	get_filename_component(module_name "${interface_header}" NAME_WE)
	set(generated_files_path "${PROJECT_BINARY_DIR}/cython/${module_name}")
	wrap_library_cython("${interface_header}" "${generated_files_path}" "${extra_imports}" "${setup_py_in_path}")
	install_cython_wrapped_library("${interface_header}" "${generated_files_path}" "${install_path}")
endfunction()


# Internal function that wraps a library and compiles the wrapper
function(wrap_library_cython interface_header generated_files_path extra_imports setup_py_in_path)
	# Wrap codegen interface
	# Extract module path and name from interface header file name
	# wrap requires interfacePath to be *absolute*
	get_filename_component(interface_header "${interface_header}" ABSOLUTE)
	get_filename_component(module_path "${interface_header}" PATH)
	get_filename_component(module_name "${interface_header}" NAME_WE)

	set(generated_cpp_file "${generated_files_path}/${module_name}.cpp")
	
	message(STATUS "Building wrap module ${module_name}")
	
	# Set up generation of module source file
	file(MAKE_DIRECTORY "${generated_files_path}")
	configure_file(${setup_py_in_path}/setup.py.in ${generated_files_path}/setup.py)
	add_custom_command(
		OUTPUT ${generated_cpp_file}
		DEPENDS ${interface_header} wrap 
        COMMAND 
            wrap --cython
            ${module_path}
            ${module_name} 
            ${generated_files_path} 
			"${extra_imports}"
			&& python setup.py build_ext --inplace
		VERBATIM
		WORKING_DIRECTORY ${generated_files_path})
		
	# Set up building of mex module
	add_custom_target(${module_name}_cython_wrapper ALL DEPENDS ${generated_cpp_file} ${interface_header})
    add_custom_target(wrap_${module_name}_cython_distclean 
	    COMMAND cmake -E remove_directory ${generated_files_path})
endfunction()

# Internal function that installs a wrap toolbox
function(install_cython_wrapped_library interface_header generated_files_path install_path)
	get_filename_component(module_name "${interface_header}" NAME_WE)

    # NOTE: only installs .pxd and .pyx and binary files (not .cpp) - the trailing slash on the directory name
	# here prevents creating the top-level module name directory in the destination.
	message(STATUS "Installing Cython Toolbox to ${install_path}") #${GTSAM_CYTHON_INSTALL_PATH}")
	if(GTSAM_BUILD_TYPE_POSTFIXES)
		foreach(build_type ${CMAKE_CONFIGURATION_TYPES})
			string(TOUPPER "${build_type}" build_type_upper)
			if(${build_type_upper} STREQUAL "RELEASE")
				set(build_type_tag "") # Don't create release mode tag on installed directory
			else()
				set(build_type_tag "${build_type}")
			endif()
			# Split up filename to strip trailing '/' in GTSAM_CYTHON_INSTALL_PATH if there is one
			get_filename_component(location "${install_path}" PATH)
			get_filename_component(name "${install_path}" NAME)
			install(DIRECTORY "${generated_files_path}/" DESTINATION "${location}/${name}${build_type_tag}" 
					CONFIGURATIONS "${build_type}" 
					PATTERN "build" EXCLUDE
					PATTERN "CMakeFiles" EXCLUDE
					PATTERN "Makefile" EXCLUDE
					PATTERN "*.cmake" EXCLUDE
					PATTERN "*.cpp" EXCLUDE
					PATTERN "*.py" EXCLUDE)
		endforeach()
	else()
		install(DIRECTORY "${generated_files_path}/" DESTINATION ${install_path} 
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

