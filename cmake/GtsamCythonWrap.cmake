# Check Cython version, need to be >=0.25.2
# Unset these cached variables to avoid surprises when the python/cython
# in the current environment are different from the cached!
unset(PYTHON_EXECUTABLE CACHE)
unset(CYTHON_EXECUTABLE CACHE)
find_package(Cython 0.25.2 REQUIRED)

# Set up cache options
set(GTSAM_CYTHON_INSTALL_PATH "" CACHE PATH "Cython toolbox destination, blank defaults to CMAKE_INSTALL_PREFIX/cython")
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
#        use "from gtsam cimport *"
# install_path: destination to install the library
# dependencies: Dependencies which need to be built before the wrapper
function(wrap_and_install_library_cython interface_header extra_imports install_path dependencies)
  # Paths for generated files
  get_filename_component(module_name "${interface_header}" NAME_WE)
  set(generated_files_path "${PROJECT_BINARY_DIR}/cython/${module_name}")
  wrap_library_cython("${interface_header}" "${generated_files_path}" "${extra_imports}" "${dependencies}")
  install_cython_wrapped_library("${interface_header}" "${generated_files_path}" "${install_path}")
endfunction()


# Internal function that wraps a library and compiles the wrapper
function(wrap_library_cython interface_header generated_files_path extra_imports dependencies)
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
  add_custom_command(
    OUTPUT ${generated_cpp_file}
    DEPENDS ${interface_header} wrap
        COMMAND
            wrap --cython
            ${module_path}
            ${module_name}
            ${generated_files_path}
      "${extra_imports}"
      && cython --cplus -I. "${generated_files_path}/${module_name}.pyx"
    VERBATIM
    WORKING_DIRECTORY ${generated_files_path}/../)
  add_custom_target(${module_name}_cython_wrapper ALL DEPENDS ${generated_cpp_file} ${interface_header} ${dependencies})

  # Set up building of cython module
  find_package(PythonLibs 2.7 REQUIRED)
  include_directories(${PYTHON_INCLUDE_DIRS})
  find_package(Eigency REQUIRED)
  include_directories(${EIGENCY_INCLUDE_DIRS})

  add_library(${module_name}_cython MODULE ${generated_cpp_file})
  set_target_properties(${module_name}_cython PROPERTIES LINK_FLAGS "-undefined dynamic_lookup"
    OUTPUT_NAME ${module_name} PREFIX "" LIBRARY_OUTPUT_DIRECTORY ${generated_files_path})
  target_link_libraries(${module_name}_cython ${module_name})
  add_dependencies(${module_name}_cython ${module_name}_cython_wrapper)

  # distclean
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
#
# Arguments:
#  source_directory: The source directory to be installed. "The last component of each directory
#                    name is appended to the destination directory but a trailing slash may be
#                    used to avoid this because it leaves the last component empty."
#                    (https://cmake.org/cmake/help/v3.3/command/install.html?highlight=install#installing-directories)
#  dest_directory: The destination directory to install to.
#  patterns: list of file patterns to install
function(install_cython_scripts source_directory dest_directory patterns)
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
      get_filename_component(location "${dest_directory}" PATH)
      get_filename_component(name "${dest_directory}" NAME)
      install(DIRECTORY "${source_directory}" DESTINATION "${location}/${name}${build_type_tag}" CONFIGURATIONS "${build_type}"
            FILES_MATCHING ${patterns_args} PATTERN "${exclude_patterns}" EXCLUDE)
    endforeach()
  else()
    install(DIRECTORY "${source_directory}" DESTINATION "${dest_directory}" FILES_MATCHING ${patterns_args} PATTERN "${exclude_patterns}" EXCLUDE)
  endif()

endfunction()

# Helper function to install specific files and handle multiple build types where the scripts
# should be installed to all build type toolboxes
#
# Arguments:
#  source_files: The source files to be installed.
#  dest_directory: The destination directory to install to.
function(install_cython_files source_files dest_directory)

  if(GTSAM_BUILD_TYPE_POSTFIXES)
    foreach(build_type ${CMAKE_CONFIGURATION_TYPES})
      string(TOUPPER "${build_type}" build_type_upper)
      if(${build_type_upper} STREQUAL "RELEASE")
        set(build_type_tag "") # Don't create release mode tag on installed directory
      else()
        set(build_type_tag "${build_type}")
      endif()
      # Split up filename to strip trailing '/' in GTSAM_CYTHON_INSTALL_PATH if there is one
      get_filename_component(location "${dest_directory}" PATH)
      get_filename_component(name "${dest_directory}" NAME)
      install(FILES "${source_files}" DESTINATION "${location}/${name}${build_type_tag}" CONFIGURATIONS "${build_type}")
    endforeach()
  else()
    install(FILES "${source_files}" DESTINATION "${dest_directory}")
  endif()

endfunction()

