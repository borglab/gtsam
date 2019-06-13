# Unset these cached variables to avoid surprises when the python in the current
# environment are different from the cached!
unset(PYTHON_EXECUTABLE CACHE)
if(GTSAM_PYTHON_VERSION STREQUAL "Default")
  find_package(PythonInterp REQUIRED)
  find_package(PythonLibs REQUIRED)
else()
  find_package(PythonInterp ${GTSAM_PYTHON_VERSION} EXACT REQUIRED)
  find_package(PythonLibs ${GTSAM_PYTHON_VERSION} EXACT REQUIRED)
endif()

# User-friendly Pybind11 wrapping and installing function. Builds a Pybind11
# module from the provided interface_header. For example, for the interface
# header gtsam.h, this will build the wrap module 'gtsam_py.cc'.
#
# Arguments:
#
# interface_header:  The relative path to the wrapper interface definition file.
# install_path: destination to install the library libs: libraries to link with
# dependencies: Dependencies which need to be built before the wrapper
function(pybind_wrap
         target
         interface_header
         generated_cpp
         module_name
         top_namespace
         ignore_classes
         libs
         dependencies)
  add_custom_command(OUTPUT ${generated_cpp}
                     COMMAND ${PYTHON_EXECUTABLE}
                             ${CMAKE_CURRENT_SOURCE_DIR}/pybind_wrapper.py
                             --src
                             ${interface_header}
                             --out
                             ${generated_cpp}
                             --module_name
                             ${module_name}
                             --top_module_namespaces
                             "${top_namespace}"
                             --ignore
                             ${ignore_classes}
                             --use_boost
                     VERBATIM)
  add_custom_target(pybind_wrap_${module_name} ALL DEPENDS ${generated_cpp})

  # Late dependency injection, to make sure this gets called whenever the
  # interface header is updated See:
  # https://stackoverflow.com/questions/40032593/cmake-does-not-rebuild-
  # dependent-after-prerequisite-changes
  add_custom_command(OUTPUT ${generated_cpp} DEPENDS ${interface_header} APPEND)

  pybind11_add_module(${target} ${generated_cpp})
  add_dependencies(${target} pybind_wrap_${module_name})
  if(NOT "${libs}" STREQUAL "")
    target_link_libraries(${target} PRIVATE "${libs}")
  endif()
  if(NOT "${dependencies}" STREQUAL "")
    add_dependencies(${target} ${dependencies})
  endif()
endfunction()

# Helper function to install Cython scripts and handle multiple build types
# where the scripts should be installed to all build type toolboxes
#
# Arguments: source_directory: The source directory to be installed. "The last
# component of each directory name is appended to the destination directory but
# a trailing slash may be used to avoid this because it leaves the last
# component empty."
# (https://cmake.org/cmake/help/v3.3/command/install.html?highlight=install
# #installing-directories) dest_directory: The destination directory to install
# to. patterns: list of file patterns to install
function(install_python_scripts
         source_directory
         dest_directory
         patterns)
  set(patterns_args "")
  set(exclude_patterns "")

  foreach(pattern ${patterns})
    list(APPEND patterns_args PATTERN "${pattern}")
  endforeach()
  if(GTSAM_BUILD_TYPE_POSTFIXES)
    foreach(build_type ${CMAKE_CONFIGURATION_TYPES})
      string(TOUPPER "${build_type}" build_type_upper)
      if(${build_type_upper} STREQUAL "RELEASE")
        set(build_type_tag "") # Don't create release mode tag on installed
                               # directory
      else()
        set(build_type_tag "${build_type}")
      endif()
      # Split up filename to strip trailing '/' in GTSAM_CYTHON_INSTALL_PATH if
      # there is one
      get_filename_component(location "${dest_directory}" PATH)
      get_filename_component(name "${dest_directory}" NAME)
      install(DIRECTORY "${source_directory}"
              DESTINATION "${location}/${name}${build_type_tag}"
              CONFIGURATIONS "${build_type}"
              FILES_MATCHING ${patterns_args}
              PATTERN "${exclude_patterns}" EXCLUDE)
    endforeach()
  else()
    install(DIRECTORY "${source_directory}"
            DESTINATION "${dest_directory}"
            FILES_MATCHING ${patterns_args}
            PATTERN "${exclude_patterns}" EXCLUDE)
  endif()

endfunction()

# Helper function to install specific files and handle multiple build types where the scripts
# should be installed to all build type toolboxes
#
# Arguments:
#  source_files: The source files to be installed.
#  dest_directory: The destination directory to install to.
function(install_python_files source_files dest_directory)

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

