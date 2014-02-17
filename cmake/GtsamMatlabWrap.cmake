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
find_program(mex_command ${mex_program_name}
	PATHS ${matlab_bin_directories} ENV PATH
	NO_DEFAULT_PATH)
mark_as_advanced(FORCE mex_command)
# Now that we have mex, trace back to find the Matlab installation root
get_filename_component(mex_command "${mex_command}" REALPATH)
get_filename_component(mex_path "${mex_command}" PATH)
get_filename_component(MATLAB_ROOT "${mex_path}/.." ABSOLUTE)
set(MATLAB_ROOT "${MATLAB_ROOT}" CACHE PATH "Path to MATLAB installation root (e.g. /usr/local/MATLAB/R2012a)")


# User-friendly wrapping function.  Builds a mex module from the provided
# interfaceHeader.  For example, for the interface header /path/to/gtsam.h,
# this will build the wrap module 'gtsam'.
# Params:
#   interfaceHeader  : Absolute or relative path to the interface definition file
#   linkLibraries    : All dependent CMake target names, library names, or full library paths
#   extraIncludeDirs : Extra include directories, in addition to those already passed to include_directories(...)
#   extraMexFlags    : Any additional compiler flags
function(wrap_and_install_library interfaceHeader linkLibraries extraIncludeDirs extraMexFlags)
	wrap_library_internal("${interfaceHeader}" "${linkLibraries}" "${extraIncludeDirs}" "${mexFlags}")
	install_wrapped_library_internal("${interfaceHeader}")
endfunction()


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
	set(generated_cpp_file "${PROJECT_BINARY_DIR}/wrap/${moduleName}/${moduleName}_wrapper.cpp")
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
	
	# Add -shared or -static suffix to targets
	set(correctedOtherLibraries "")
	set(otherLibraryTargets "")
    set(otherLibraryNontargets "")
	foreach(lib ${moduleName} ${linkLibraries})
		if(TARGET ${lib})
			list(APPEND correctedOtherLibraries ${lib})
			list(APPEND otherLibraryTargets ${lib})
		elseif(TARGET ${lib}-shared) # Prefer the shared library if we have both shared and static)
			list(APPEND correctedOtherLibraries ${lib}-shared)
			list(APPEND otherLibraryTargets ${lib}-shared)
		elseif(TARGET ${lib}-static)
			list(APPEND correctedOtherLibraries ${lib}-static)
			list(APPEND otherLibraryTargets ${lib}-static)
		else()
			list(APPEND correctedOtherLibraries ${lib})
            list(APPEND otherLibraryNontargets ${lib})
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
		DEPENDS ${interfaceHeader} wrap ${module_library_target} ${otherLibraryTargets}
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
	add_library(${moduleName}_wrapper MODULE ${generated_cpp_file} ${interfaceHeader})
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

function(install_wrapped_library_internal interfaceHeader)
	get_filename_component(moduleName "${interfaceHeader}" NAME_WE)
	set(generated_files_path "${PROJECT_BINARY_DIR}/wrap/${moduleName}")

    # NOTE: only installs .m and mex binary files (not .cpp) - the trailing slash on the directory name
	# here prevents creating the top-level module name directory in the destination.
	message(STATUS "Installing Matlab Toolbox to ${GTSAM_TOOLBOX_INSTALL_PATH}")
	if(GTSAM_BUILD_TYPE_POSTFIXES)
		foreach(build_type ${CMAKE_CONFIGURATION_TYPES})
			string(TOUPPER "${build_type}" build_type_upper)
			if("${build_type_upper}" STREQUAL "RELEASE")
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


# Function to setup codegen and building of the wrap toolbox
#
# params:
#  moduleName       : the name of the module, interface file must be called moduleName.h
#  mexFlags         : Compilation flags to be passed to the mex compiler 
#  modulePath       : relative path to module markup header file (called moduleName.h)
#  otherLibraries   : list of library targets this should depend on
#  toolboxPath      : the directory in which to generate/build wrappers
#  wrap_header_path : path to the installed wrap header  
function(wrap_library_generic moduleName mexFlags modulePath otherLibraries toolbox_path wrap_header_path)

	if(NOT "${CMAKE_PROJECT_NAME}" STREQUAL "GTSAM")
		message("Your project uses wrap_library or wrap_library_generic - this is deprecated, please use the more user-friendly function wrap_and_install_library")
	endif()

	# Append module name to link libraries to keep original behavior
	list(APPEND otherLibraries ${moduleName})
	
	# Set up arguments
	set(interfaceHeader ${modulePath}/${moduleName}.h)
	
	# Call internal function
	wrap_library_internal("${interfaceHeader}" "${otherLibraries}" "" "${mexFlags}")
endfunction(wrap_library_generic)

# Function to setup codegen, building and installation of the wrap toolbox
# This wrap setup function assumes that the toolbox will be installed directly, 
# with predictable matlab.h sourcing.  Use this version when the toolbox will be used
# from the installed version, rather than in place.  
# Assumes variable GTSAM_WRAP_HEADER_PATH has been set
# params:
#  moduleName      : the name of the module, interface file must be called moduleName.h
#  mexFlags        : Compilation flags to be passed to the mex compiler 
#  modulePath      : relative path to module markup header file (called moduleName.h)
#  otherLibraries  : list of library targets this should depend on
function(wrap_library moduleName mexFlags modulePath otherLibraries)
    # Toolbox generation path goes in build folder
    set(toolbox_base_path ${PROJECT_BINARY_DIR}/wrap)
    set(toolbox_path ${toolbox_base_path}/${moduleName})
    
    # Call generic version of function
    wrap_library_generic("${moduleName}" "${mexFlags}" "${modulePath}" "${otherLibraries}" "${toolbox_path}" "${GTSAM_WRAP_HEADER_PATH}")
	
	install_wrapped_library_internal("${modulePath}/${moduleName}.h")
    
endfunction(wrap_library)

# Helper function to install MATLAB scripts and handle multiple build types where the scripts
# should be installed to all build type toolboxes
function(install_matlab_scripts source_directory patterns)
	set(patterns_args "")
	foreach(pattern ${patterns})
		list(APPEND patterns_args PATTERN "${pattern}")
	endforeach()
	if(GTSAM_BUILD_TYPE_POSTFIXES)
		foreach(build_type ${CMAKE_CONFIGURATION_TYPES})
			string(TOUPPER "${build_type}" build_type_upper)
			if("${build_type_upper}" STREQUAL "RELEASE")
				set(build_type_tag "") # Don't create release mode tag on installed directory
			else()
				set(build_type_tag "${build_type}")
			endif()
			# Split up filename to strip trailing '/' in GTSAM_TOOLBOX_INSTALL_PATH if there is one
			get_filename_component(location "${GTSAM_TOOLBOX_INSTALL_PATH}" PATH)
			get_filename_component(name "${GTSAM_TOOLBOX_INSTALL_PATH}" NAME)
			install(DIRECTORY "${source_directory}" DESTINATION "${location}/${name}${build_type_tag}" CONFIGURATIONS "${build_type}" FILES_MATCHING ${patterns_args} PATTERN ".svn" EXCLUDE)
		endforeach()
	else()
		install(DIRECTORY "${source_directory}" DESTINATION "${GTSAM_TOOLBOX_INSTALL_PATH}" FILES_MATCHING ${patterns_args} PATTERN ".svn" EXCLUDE)
	endif()

endfunction()

