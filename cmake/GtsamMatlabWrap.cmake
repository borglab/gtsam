# Set up cache options
option(GTSAM_MEX_BUILD_STATIC_MODULE "Build MATLAB wrapper statically (increases build time)" OFF)
set(GTSAM_BUILD_MEX_BINARY_FLAGS "" CACHE STRING "Extra flags for running Matlab MEX compilation")
set(GTSAM_TOOLBOX_INSTALL_PATH "" CACHE PATH "Matlab toolbox destination, blank defaults to CMAKE_INSTALL_PREFIX/gtsam_toolbox")
if(NOT GTSAM_TOOLBOX_INSTALL_PATH)
	set(GTSAM_TOOLBOX_INSTALL_PATH "${CMAKE_INSTALL_PREFIX}/gtsam_toolbox")
endif()

# GTSAM_MEX_BUILD_STATIC_MODULE is not for Windows - on Windows any static
# are already compiled into the library by the linker
if(GTSAM_MEX_BUILD_STATIC_MODULE AND WIN32)
	message(FATAL_ERROR "GTSAM_MEX_BUILD_STATIC_MODULE should not be set on Windows - the linker already automatically compiles in any dependent static libraries.  To create a standalone toolbox pacakge, simply ensure that CMake finds the static versions of all dependent libraries (Boost, etc).")
endif()

# Try to automatically configure mex path
if(APPLE)
	file(GLOB matlab_bin_directories "/Applications/MATLAB*/bin")
	set(mex_program_name "mex")
elseif(WIN32)
	file(GLOB matlab_bin_directories "C:/Program Files*/MATLAB/*/bin")
	set(mex_program_name "mex.bat")
else()
	file(GLOB matlab_bin_directories "/usr/local/MATLAB/*/bin")
	set(mex_program_name "mex")
endif()
# Run find_program explicitly putting $PATH after our predefined program
# directories using 'ENV PATH' and 'NO_SYSTEM_ENVIRONMENT_PATH' - this prevents
# finding the LaTeX mex program (totally unrelated to MATLAB Mex) when LaTeX is
# on the system path.
list(REVERSE matlab_bin_directories) # Reverse list so the highest version (sorted alphabetically) is preferred
find_program(MEX_COMMAND ${mex_program_name}
	PATHS ${matlab_bin_directories} ENV PATH
	NO_DEFAULT_PATH)
mark_as_advanced(FORCE MEX_COMMAND)
# Now that we have mex, trace back to find the Matlab installation root
get_filename_component(MEX_COMMAND "${MEX_COMMAND}" REALPATH)
get_filename_component(mex_path "${MEX_COMMAND}" PATH)
get_filename_component(MATLAB_ROOT "${mex_path}/.." ABSOLUTE)
set(MATLAB_ROOT "${MATLAB_ROOT}" CACHE PATH "Path to MATLAB installation root (e.g. /usr/local/MATLAB/R2012a)")


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
# extraMexFlags:    Any *additional* flags to pass to the compiler when building
#                   the wrap code.  Normally, leave this empty.
function(wrap_and_install_library interfaceHeader linkLibraries extraIncludeDirs extraMexFlags)
	wrap_library_internal("${interfaceHeader}" "${linkLibraries}" "${extraIncludeDirs}" "${mexFlags}")
	install_wrapped_library_internal("${interfaceHeader}")
endfunction()


# Internal function that wraps a library and compiles the wrapper
function(wrap_library_internal interfaceHeader linkLibraries extraIncludeDirs extraMexFlags)
	if(UNIX AND NOT APPLE)
		if(CMAKE_SIZEOF_VOID_P EQUAL 8)
			set(mexModuleExt mexa64)
		else()
			set(mexModuleExt mexglx)
		endif()
	elseif(APPLE)
		set(mexModuleExt mexmaci64)
	elseif(MSVC)
		if(CMAKE_CL_64)
			set(mexModuleExt mexw64)
		else()
			set(mexModuleExt mexw32)
		endif()
	endif()
		
	# Wrap codegen interface
    #usage: wrap interfacePath moduleName toolboxPath headerPath
    #  interfacePath : *absolute* path to directory of module interface file
    #  moduleName    : the name of the module, interface file must be called moduleName.h
    #  toolboxPath   : the directory in which to generate the wrappers
    #  headerPath    : path to matlab.h 
	
	# Extract module name from interface header file name
	get_filename_component(interfaceHeader "${interfaceHeader}" ABSOLUTE)
	get_filename_component(modulePath "${interfaceHeader}" PATH)
	get_filename_component(moduleName "${interfaceHeader}" NAME_WE)
	
	# Paths for generated files
	set(generated_files_path "${PROJECT_BINARY_DIR}/wrap/${moduleName}")
	set(generated_cpp_file "${generated_files_path}/${moduleName}_wrapper.cpp")
	set(compiled_mex_modules_root "${PROJECT_BINARY_DIR}/wrap/${moduleName}_mex")
	
	message(STATUS "Building wrap module ${moduleName}")
	
	# Find matlab.h in GTSAM
	if("${PROJECT_NAME}" STREQUAL "GTSAM")
		set(matlab_h_path "${PROJECT_SOURCE_DIR}")
	else()
		if(NOT GTSAM_INCLUDE_DIR)
			message(FATAL_ERROR "You must call find_package(GTSAM) before using wrap")
		endif()
		list(GET GTSAM_INCLUDE_DIR 0 installed_includes_path)
		set(matlab_h_path "${installed_includes_path}/wrap")
	endif()

	# If building a static mex module, add all cmake-linked libraries to the
	# explicit link libraries list so that the next block of code can unpack
	# any static libraries
	set(automaticDependencies "")
	foreach(lib ${moduleName} ${linkLibraries})
	  #message("MODULE NAME: ${moduleName}")
		if(TARGET "${lib}")
            get_target_property(dependentLibraries ${lib} INTERFACE_LINK_LIBRARIES)
           # message("DEPENDENT LIBRARIES:  ${dependentLibraries}")
            if(dependentLibraries)
               list(APPEND automaticDependencies ${dependentLibraries})
            endif()
        endif()
    endforeach()
    
    ## CHRIS: Temporary fix. On my system the get_target_property above returned Not-found for gtsam module
    ## This needs to be fixed!!
    if(UNIX AND NOT APPLE)
      list(APPEND automaticDependencies ${Boost_SERIALIZATION_LIBRARY_RELEASE} ${Boost_FILESYSTEM_LIBRARY_RELEASE}
        ${Boost_SYSTEM_LIBRARY_RELEASE} ${Boost_THREAD_LIBRARY_RELEASE} ${Boost_DATE_TIME_LIBRARY_RELEASE}
        ${Boost_REGEX_LIBRARY_RELEASE})
      if(Boost_TIMER_LIBRARY_RELEASE AND NOT GTSAM_DISABLE_NEW_TIMERS) # Only present in Boost >= 1.48.0
        list(APPEND automaticDependencies ${Boost_TIMER_LIBRARY_RELEASE} ${Boost_CHRONO_LIBRARY_RELEASE})
        if(GTSAM_MEX_BUILD_STATIC_MODULE)
          #list(APPEND automaticDependencies -Wl,--no-as-needed -lrt)
        endif()
      endif()
    endif()
    
    #message("AUTOMATIC DEPENDENCIES:  ${automaticDependencies}")
    ## CHRIS: End temporary fix

	# Separate dependencies
	set(correctedOtherLibraries "")
	set(otherLibraryTargets "")
    set(otherLibraryNontargets "")
    set(otherSourcesAndObjects "")
	foreach(lib ${moduleName} ${linkLibraries} ${automaticDependencies})
		if(TARGET "${lib}")
		    if(GTSAM_MEX_BUILD_STATIC_MODULE)
		    	get_target_property(target_sources ${lib} SOURCES)
		    	list(APPEND otherSourcesAndObjects ${target_sources})
		    else()
				list(APPEND correctedOtherLibraries ${lib})
				list(APPEND otherLibraryTargets ${lib})
		    endif()
		else()
			get_filename_component(file_extension "${lib}" EXT)
			get_filename_component(lib_name "${lib}" NAME_WE)
			if(file_extension STREQUAL ".a" AND GTSAM_MEX_BUILD_STATIC_MODULE)
				# For building a static MEX module, unpack the static library
				# and compile its object files into our module
				file(MAKE_DIRECTORY "${generated_files_path}/${lib_name}_objects")
				execute_process(COMMAND ar -x "${lib}"
				                WORKING_DIRECTORY "${generated_files_path}/${lib_name}_objects"
				                RESULT_VARIABLE ar_result)
				if(NOT ar_result EQUAL 0)
					message(FATAL_ERROR "Failed extracting ${lib}")
				endif()

				# Get list of object files
				execute_process(COMMAND ar -t "${lib}"
								OUTPUT_VARIABLE object_files
				                RESULT_VARIABLE ar_result)
				if(NOT ar_result EQUAL 0)
					message(FATAL_ERROR "Failed listing ${lib}")
				endif()

				# Add directory to object files
				string(REPLACE "\n" ";" object_files_list "${object_files}")
				foreach(object_file ${object_files_list})
					get_filename_component(file_extension "${object_file}" EXT)
					if(file_extension STREQUAL ".o")
						list(APPEND otherSourcesAndObjects "${generated_files_path}/${lib_name}_objects/${object_file}")
					endif()
				endforeach()
			else()
				list(APPEND correctedOtherLibraries ${lib})
	            list(APPEND otherLibraryNontargets ${lib})
            endif()
		endif()
	endforeach()
    
    # Check libraries for conflicting versions built-in to MATLAB
    set(dependentLibraries "")
    if(NOT "${otherLibraryTargets}" STREQUAL "")
        foreach(target ${otherLibraryTargets})
            get_target_property(dependentLibrariesOne ${target} INTERFACE_LINK_LIBRARIES)
            list(APPEND dependentLibraries ${dependentLibrariesOne})
        endforeach()
    endif()
    list(APPEND dependentLibraries ${otherLibraryNontargets})
    check_conflicting_libraries_internal("${dependentLibraries}")

	# Set up generation of module source file
	file(MAKE_DIRECTORY "${generated_files_path}")
	add_custom_command(
		OUTPUT ${generated_cpp_file}
		DEPENDS ${interfaceHeader} wrap ${module_library_target} ${otherLibraryTargets} ${otherSourcesAndObjects}
        COMMAND 
            wrap
            ${modulePath}
            ${moduleName} 
            ${generated_files_path} 
            ${matlab_h_path} 
		VERBATIM
		WORKING_DIRECTORY ${generated_files_path})
		
	# Set up building of mex module
	string(REPLACE ";" " " extraMexFlagsSpaced "${extraMexFlags}")
	string(REPLACE ";" " " mexFlagsSpaced "${GTSAM_BUILD_MEX_BINARY_FLAGS}")
	add_library(${moduleName}_wrapper MODULE ${generated_cpp_file} ${interfaceHeader} ${otherSourcesAndObjects})
	target_link_libraries(${moduleName}_wrapper ${correctedOtherLibraries})
	set_target_properties(${moduleName}_wrapper PROPERTIES
		OUTPUT_NAME              "${moduleName}_wrapper"
		PREFIX                   ""
		SUFFIX                   ".${mexModuleExt}"
		LIBRARY_OUTPUT_DIRECTORY "${compiled_mex_modules_root}"
		ARCHIVE_OUTPUT_DIRECTORY "${compiled_mex_modules_root}"
		RUNTIME_OUTPUT_DIRECTORY "${compiled_mex_modules_root}"
		CLEAN_DIRECT_OUTPUT 1)
	set_property(TARGET ${moduleName}_wrapper APPEND_STRING PROPERTY COMPILE_FLAGS " ${extraMexFlagsSpaced} ${mexFlagsSpaced} \"-I${MATLAB_ROOT}/extern/include\" -DMATLAB_MEX_FILE -DMX_COMPAT_32")
	set_property(TARGET ${moduleName}_wrapper APPEND PROPERTY INCLUDE_DIRECTORIES ${extraIncludeDirs})
	# Disable build type postfixes for the mex module - we install in different directories for each build type instead
	foreach(build_type ${CMAKE_CONFIGURATION_TYPES})
		string(TOUPPER "${build_type}" build_type_upper)
		set_target_properties(${moduleName}_wrapper PROPERTIES ${build_type_upper}_POSTFIX "")
	endforeach()
	# Set up platform-specific flags
	if(MSVC)
		if(CMAKE_CL_64)
			set(mxLibPath "${MATLAB_ROOT}/extern/lib/win64/microsoft")
		else()
			set(mxLibPath "${MATLAB_ROOT}/extern/lib/win32/microsoft")
		endif()
		target_link_libraries(${moduleName}_wrapper "${mxLibPath}/libmex.lib" "${mxLibPath}/libmx.lib" "${mxLibPath}/libmat.lib")
		set_target_properties(${moduleName}_wrapper PROPERTIES LINK_FLAGS "/export:mexFunction")
		set_property(SOURCE "${generated_cpp_file}" APPEND PROPERTY COMPILE_FLAGS "/bigobj")
	elseif(APPLE)
		set(mxLibPath "${MATLAB_ROOT}/bin/maci64")
		target_link_libraries(${moduleName}_wrapper "${mxLibPath}/libmex.dylib" "${mxLibPath}/libmx.dylib" "${mxLibPath}/libmat.dylib")
	endif()

    # Hacking around output issue with custom command
    # Deletes generated build folder 
    add_custom_target(wrap_${moduleName}_distclean 
	    COMMAND cmake -E remove_directory ${generated_files_path}
		COMMAND cmake -E remove_directory ${compiled_mex_modules_root})
endfunction()

# Internal function that installs a wrap toolbox
function(install_wrapped_library_internal interfaceHeader)
	get_filename_component(moduleName "${interfaceHeader}" NAME_WE)
	set(generated_files_path "${PROJECT_BINARY_DIR}/wrap/${moduleName}")

    # NOTE: only installs .m and mex binary files (not .cpp) - the trailing slash on the directory name
	# here prevents creating the top-level module name directory in the destination.
	message(STATUS "Installing Matlab Toolbox to ${GTSAM_TOOLBOX_INSTALL_PATH}")
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
			install(DIRECTORY "${generated_files_path}/" DESTINATION "${location}/${name}${build_type_tag}" CONFIGURATIONS "${build_type}" FILES_MATCHING PATTERN "*.m")
			install(TARGETS ${moduleName}_wrapper
				LIBRARY DESTINATION "${location}/${name}${build_type_tag}" CONFIGURATIONS "${build_type}"
				RUNTIME DESTINATION "${location}/${name}${build_type_tag}" CONFIGURATIONS "${build_type}")
		endforeach()
	else()
		install(DIRECTORY "${generated_files_path}/" DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH} FILES_MATCHING PATTERN "*.m")
		install(TARGETS ${moduleName}_wrapper
			LIBRARY DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH}
			RUNTIME DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH})
	endif()
endfunction()

# Internal function to check for libraries installed with MATLAB that may conflict
# and prints a warning to move them if problems occur.
function(check_conflicting_libraries_internal libraries)
    if(UNIX)
        # Set path for matlab's built-in libraries
        if(APPLE)
            set(mxLibPath "${MATLAB_ROOT}/bin/maci64")
        else()
            if(CMAKE_CL_64)
                set(mxLibPath "${MATLAB_ROOT}/bin/glnxa64")
            else()
                set(mxLibPath "${MATLAB_ROOT}/bin/glnx86")
            endif()
        endif()
        
        # List matlab's built-in libraries
        file(GLOB matlabLibs RELATIVE "${mxLibPath}" "${mxLibPath}/lib*")
        
        # Convert to base names
        set(matlabLibNames "")
        foreach(lib ${matlabLibs})
            get_filename_component(libName "${lib}" NAME_WE)
            list(APPEND matlabLibNames "${libName}")
        endforeach()
        
        # Get names of link libraries
        set(linkLibNames "")
        foreach(lib ${libraries})
            string(FIND "${lib}" "/" slashPos)
            if(NOT slashPos EQUAL -1)
                # If the name is a path, just get the library name
                get_filename_component(libName "${lib}" NAME_WE)
                list(APPEND linkLibNames "${libName}")
            else()
                # It's not a path, so see if it looks like a filename
                get_filename_component(ext "${lib}" EXT)
                if(NOT "${ext}" STREQUAL "")
                    # It's a filename, so get the base name
                    get_filename_component(libName "${lib}" NAME_WE)
                    list(APPEND linkLibNames "${libName}")
                else()
                    # It's not a filename so it must be a short name, add the "lib" prefix
                    list(APPEND linkLibNames "lib${lib}")
                endif()
            endif()
        endforeach()
        
        # Remove duplicates
        list(REMOVE_DUPLICATES linkLibNames)
        
        set(conflictingLibs "")
        foreach(lib ${linkLibNames})
            list(FIND matlabLibNames "${lib}" libPos)
            if(NOT libPos EQUAL -1)
                if(NOT conflictingLibs STREQUAL "")
                    set(conflictingLibs "${conflictingLibs}, ")
                endif()
                set(conflictingLibs "${conflictingLibs}${lib}")
            endif()
        endforeach()
        
        if(NOT "${conflictingLibs}" STREQUAL "")
            message(WARNING "GTSAM links to the libraries [ ${conflictingLibs} ] on your system, but "
                "MATLAB is distributed with its own versions of these libraries which may conflict. "
                "If you get strange errors or crashes with the GTSAM MATLAB wrapper, move these "
                "libraries out of MATLAB's built-in library directory, which is ${mxLibPath} on "
                "your system.  MATLAB will usually still work with these libraries moved away, but "
                "if not, you'll have to compile the static GTSAM MATLAB wrapper module.")
        endif()
    endif()
endfunction()

# Helper function to install MATLAB scripts and handle multiple build types where the scripts
# should be installed to all build type toolboxes
function(install_matlab_scripts source_directory patterns)
	set(patterns_args "")
	set(exclude_patterns "")
	if(NOT GTSAM_WRAP_SERIALIZATION)
	    set(exclude_patterns "testSerialization.m")
	endif()

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
			# Split up filename to strip trailing '/' in GTSAM_TOOLBOX_INSTALL_PATH if there is one
			get_filename_component(location "${GTSAM_TOOLBOX_INSTALL_PATH}" PATH)
			get_filename_component(name "${GTSAM_TOOLBOX_INSTALL_PATH}" NAME)
			install(DIRECTORY "${source_directory}" DESTINATION "${location}/${name}${build_type_tag}" CONFIGURATIONS "${build_type}" FILES_MATCHING ${patterns_args} PATTERN "${exclude_patterns}" EXCLUDE)
		endforeach()
	else()
		install(DIRECTORY "${source_directory}" DESTINATION "${GTSAM_TOOLBOX_INSTALL_PATH}" FILES_MATCHING ${patterns_args} PATTERN "${exclude_patterns}" EXCLUDE)
	endif()

endfunction()

