# Set up cache options
option(GTSAM_MEX_BUILD_STATIC_MODULE "Build MATLAB wrapper statically (increases build time)" OFF)
set(GTSAM_BUILD_MEX_BINARY_FLAGS "" CACHE STRING "Extra flags for running Matlab MEX compilation")
set(GTSAM_TOOLBOX_INSTALL_PATH "" CACHE PATH "Matlab toolbox destination, blank defaults to CMAKE_INSTALL_PREFIX/borg/toolbox")
if(NOT GTSAM_TOOLBOX_INSTALL_PATH)
	set(GTSAM_TOOLBOX_INSTALL_PATH "${CMAKE_INSTALL_PREFIX}/borg/toolbox")
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
	DOC "Command to use for executing mex (if on path, 'mex' will work)"
	NO_SYSTEM_ENVIRONMENT_PATH)
	

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
    
    set(module_markup_header_file "${moduleName}.h")
    set(module_markup_header_path "${CMAKE_CURRENT_SOURCE_DIR}/${modulePath}/${module_markup_header_file}")
	
	# Add wrap cpp file first
	set(mexSources "")
	list(APPEND mexSources ${moduleName}_wrapper.cpp)
	
	# Add boost libraries
	string(TOUPPER ${CMAKE_BUILD_TYPE} build_type_toupper)
	if("${build_type_toupper}" STREQUAL "DEBUG")
		list(APPEND otherLibraries ${Boost_SERIALIZATION_LIBRARY_DEBUG} ${Boost_FILESYSTEM_LIBRARY_DEBUG}
			${Boost_SYSTEM_LIBRARY_DEBUG} ${Boost_THREAD_LIBRARY_DEBUG} ${Boost_DATE_TIME_LIBRARY_DEBUG}
			${Boost_REGEX_LIBRARY_DEBUG})
		if(Boost_TIMER_LIBRARY_DEBUG AND NOT GTSAM_DISABLE_NEW_TIMERS) # Only present in Boost >= 1.48.0
			list(APPEND otherLibraries ${Boost_TIMER_LIBRARY_DEBUG} ${Boost_CHRONO_LIBRARY_DEBUG})
			if(GTSAM_MEX_BUILD_STATIC_MODULE)
				list(APPEND otherLibraries -Wl,--no-as-needed -lrt)
			endif()
		endif()
	else()
		list(APPEND otherLibraries ${Boost_SERIALIZATION_LIBRARY_RELEASE} ${Boost_FILESYSTEM_LIBRARY_RELEASE}
			${Boost_SYSTEM_LIBRARY_RELEASE} ${Boost_THREAD_LIBRARY_RELEASE} ${Boost_DATE_TIME_LIBRARY_RELEASE}
			${Boost_REGEX_LIBRARY_RELEASE})
		if(Boost_TIMER_LIBRARY_RELEASE AND NOT GTSAM_DISABLE_NEW_TIMERS) # Only present in Boost >= 1.48.0
			list(APPEND otherLibraries ${Boost_TIMER_LIBRARY_RELEASE} ${Boost_CHRONO_LIBRARY_RELEASE})
			if(GTSAM_MEX_BUILD_STATIC_MODULE)
				list(APPEND otherLibraries -Wl,--no-as-needed -lrt)
			endif()
		endif()
	endif()

	# Sort otherLibraries into files and targets, add -shared or -static to other dependency library names
	set(otherLibraryTargets "")
	set(otherLibraryLinks "")
	set(otherLibraryArchives "")
	foreach(lib ${moduleName} ${otherLibraries})
		if(TARGET ${lib})
			list(APPEND otherLibraryTargets ${lib})
		elseif(TARGET ${lib}-shared) # Prefer the shared library if we have both shared and static
			list(APPEND otherLibraryTargets ${lib}-shared)
		elseif(TARGET ${lib}-static)
			list(APPEND otherLibraryTargets ${lib}-static)
		else()
			if(${lib} MATCHES "^.+\\.a$")
				list(APPEND otherLibraryArchives ${lib})
			else()
				list(APPEND otherLibraryLinks ${lib})
			endif()
		endif()
	endforeach()
	
	# Get path to libraries with current build type
	foreach(target ${otherLibraryTargets})
		if(GTSAM_MEX_BUILD_STATIC_MODULE)
			# If building the wrapper statically, add the target's sources to the mex sources
			get_target_property(librarySources ${target} SOURCES)
			list(APPEND mexSources ${librarySources})
		else()
			# If not building wrapper staticically, link to the target library
      get_target_property(libraryFile ${target} LOCATION_${build_type_toupper})
      get_target_property(targetType ${target} TYPE)
      if(WIN32 AND NOT targetType STREQUAL STATIC_LIBRARY)
        # On Windows with a shared library, we link to the import library file
        get_filename_component(libraryDir ${libraryFile} PATH)
        get_filename_component(libraryName ${libraryFile} NAME_WE)
		get_filename_component(oneUpDir ${libraryDir} NAME)
		string(TOUPPER "${oneUpDir}" oneUpDir)
		if("${oneUpDir}" STREQUAL BIN OR "${oneUpDir}" STREQUAL LIB)
			# One level up is 'bin' or 'lib' so we do not have an extra
			# subdirectory for the build configuration.
			set(libraryFile "${libraryDir}/../lib/${libraryName}.lib")
		else()
			# One level up was neither 'bin' nor 'lib' so assume we also
			# have a subdirectory for the build configuration.
			set(libraryFile "${libraryDir}/../../lib/${build_type_toupper}/${libraryName}.lib")
		endif()
      endif()
			list(APPEND otherLibraryLinks ${libraryFile})
		endif()
	endforeach()
	
	# Bad hack - if static wrapper and this is GTSAM, add colamd to the include path
    if(GTSAM_MEX_BUILD_STATIC_MODULE AND (${moduleName} STREQUAL "gtsam" OR ${moduleName} STREQUAL "gtsam_unstable"))
		list(APPEND mexFlags
			"-I${PROJECT_SOURCE_DIR}/gtsam/3rdparty/CCOLAMD/Include" "-I${PROJECT_SOURCE_DIR}/gtsam/3rdparty/UFconfig")
	endif()
	
	# If using Boost shared libs, set up auto linking for shared libs
	if(NOT Boost_USE_STATIC_LIBS)
		list(APPEND mexFlags "-DBOOST_ALL_DYN_LINK")
	endif()

	# Add the CXXFLAGS from building the library
	if(MSVC)
		set(compile_flag_var "COMPFLAGS")
		set(link_flag_var "LINKFLAGS")
		set(cxx_flags_extra "/bigobj")
	else()
		set(compile_flag_var "CXXFLAGS")
		set(link_flag_var "LDFLAGS")
		set(cxx_flags_extra "")
	endif()
	list(APPEND mexFlags "${compile_flag_var}=$${compile_flag_var} ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${build_type_toupper}} ${cxx_flags_extra}")
	
	# If building a static module on UNIX, extract dependent .a libraries and add their .o files
	if(UNIX AND GTSAM_MEX_BUILD_STATIC_MODULE)
		file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/wrap/external_archives)
		foreach(archive ${otherLibraryArchives})
			get_filename_component(archive_name ${archive} NAME_WE)
			set(extraction_dir ${CMAKE_BINARY_DIR}/wrap/external_archives/${archive_name})
			file(MAKE_DIRECTORY ${extraction_dir})
			# Extract archive
			execute_process(
				COMMAND ar x ${archive}
				WORKING_DIRECTORY ${extraction_dir})
			# Get list of .o's in archive
			execute_process(
				COMMAND ar t ${archive}
				OUTPUT_VARIABLE object_files OUTPUT_STRIP_TRAILING_WHITESPACE)
			string(REPLACE "\n" ";" object_list ${object_files})
			set(object_list_absolute "")
			foreach(object ${object_list})
				list(APPEND object_list_absolute ${extraction_dir}/${object})
			endforeach()
			list(APPEND mexSources ${object_list_absolute})
		endforeach()
		set(otherLibraryArchives "")
	endif()

	# Add libraries to command line
	string(REPLACE ";" " " otherLibraryArchivesSpaced "${otherLibraryArchives}")
	list(APPEND mexFlags "${link_flag_var}=$${link_flag_var} ${otherLibraryArchivesSpaced}")
	list(APPEND mexFlags ${otherLibraryLinks})
	
	# Add -g if debug mode
	if(${build_type_toupper} STREQUAL DEBUG OR ${build_type_toupper} STREQUAL RELWITHDEBINFO)
		list(APPEND mexFlags -g)
	endif()

	# Verbose mex - print mex compilation flags to help notice flag problems
	list(APPEND mexFlags -v)
	
	# Add sources
	set(mexSourcesFiltered "")
	foreach(src ${mexSources})
		if(src MATCHES "^.+\\.cpp$" OR src MATCHES "^.+\\.c$" OR src MATCHES "^.+\\.o$")
			list(APPEND mexSourcesFiltered ${src})
		endif()
	endforeach()
	set(mexFlags ${mexSourcesFiltered} ${mexFlags})
		
	string(REPLACE ";" "' '" mexFlagsForDisplay "${mexFlags}")
    message(STATUS "Building Matlab Wrapper, mexFlags: [ '${mexFlagsForDisplay}' ]")
    
	# Wrap codegen interface
    #usage: wrap interfacePath moduleName toolboxPath headerPath
    #  interfacePath : *absolute* path to directory of module interface file
    #  moduleName    : the name of the module, interface file must be called moduleName.h
    #  toolboxPath   : the directory in which to generate the wrappers
    #  headerPath    : path to matlab.h 
	    
    # Code generation command
    # FIXME: doesn't get cleaned properly because cmake doesn't track the dependency
    # TODO: convert to a custom_command with output specified
	get_target_property(wrap_LOCATION wrap LOCATION_${CMAKE_BUILD_TYPE})
    add_custom_target(wrap_${moduleName} 
        ALL             # Forces wrap to always be run, but no files are touched if they don't change  
        COMMAND 
            ${wrap_LOCATION}
            ${CMAKE_CURRENT_SOURCE_DIR}/${modulePath}
            ${moduleName} 
            ${toolbox_path} 
            ${wrap_header_path} 
			VERBATIM)
    add_dependencies(wrap_${moduleName} wrap ${module_markup_header_path})
    
    # Hacking around output issue with custom command
    # Deletes generated build folder 
    add_custom_target(wrap_${moduleName}_distclean 
	COMMAND
		cmake -E remove_directory ${toolbox_path})

	# Make building mex binaries a part of the all (and install) target 
	set(build_all "ALL")
	
	# If building wrapper statically, we compile all the library sources
	# with mex, so need no extra dependencies, but if not building statically,
	# we depend on the libraries since we will link to them.
	if(GTSAM_MEX_BUILD_STATIC_MODULE)
		set(wrap_build_dependencies "")
	else()
		set(wrap_build_dependencies ${moduleTarget} ${otherLibraryTargets})
	endif()

	# Actual build target for building mex files
	add_custom_target(wrap_${moduleName}_build ${build_all}
		COMMAND "${MEX_COMMAND}" ${mexFlags}
		VERBATIM
		WORKING_DIRECTORY ${toolbox_path})
	add_dependencies(wrap_${moduleName}_build wrap_${moduleName} ${wrap_build_dependencies})

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
    set(toolbox_base_path ${CMAKE_BINARY_DIR}/wrap)
    set(toolbox_path ${toolbox_base_path}/${moduleName})
    
    # Call generic version of function
    wrap_library_generic("${moduleName}" "${mexFlags}" "${modulePath}" "${otherLibraries}" "${toolbox_path}" "${GTSAM_WRAP_HEADER_PATH}")
    
    # NOTE: only installs .m and mex binary files (not .cpp) - the trailing slash on the directory name
	# here prevents creating the top-level module name directory in the destination.
	message(STATUS "Installing Matlab Toolbox to ${GTSAM_TOOLBOX_INSTALL_PATH}")
	install(DIRECTORY ${toolbox_path}/ DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH} FILES_MATCHING PATTERN "*.m")
	install(DIRECTORY ${toolbox_path}/ DESTINATION ${GTSAM_TOOLBOX_INSTALL_PATH} FILES_MATCHING PATTERN "*.mex*")
endfunction(wrap_library)
