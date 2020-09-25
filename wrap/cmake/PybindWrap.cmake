# Unset these cached variables to avoid surprises when the python in the current
# environment are different from the cached!
unset(PYTHON_EXECUTABLE CACHE)
unset(PYTHON_INCLUDE_DIR CACHE)
unset(PYTHON_MAJOR_VERSION CACHE)

# Allow override from command line
if(NOT DEFINED WRAP_USE_CUSTOM_PYTHON_LIBRARY)
  if(WRAP_PYTHON_VERSION STREQUAL "Default")
    find_package(PythonInterp REQUIRED)
    find_package(PythonLibs REQUIRED)
  else()
    find_package(PythonInterp
                ${WRAP_PYTHON_VERSION}
                EXACT
                REQUIRED)
    find_package(PythonLibs
                ${WRAP_PYTHON_VERSION}
                EXACT
                REQUIRED)
  endif()
endif()

set(DIR_OF_WRAP_PYBIND_CMAKE ${CMAKE_CURRENT_LIST_DIR})

set(PYBIND11_PYTHON_VERSION ${WRAP_PYTHON_VERSION})

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../pybind11 pybind11)

# User-friendly Pybind11 wrapping and installing function.
# Builds a Pybind11 module from the provided interface_header.
# For example, for the interface header gtsam.h, this will
# build the wrap module 'gtsam_py.cc'.
#
# Arguments:
# ~~~
# target: The Make target
# interface_header:  The relative path to the wrapper interface definition file.
# generated_cpp: The name of the cpp file which is generated from the tpl file.
# module_name: The name of the Python module to use.
# top_namespace: The C++ namespace under which the code to be wrapped exists.
# ignore_classes: CMake list of classes to ignore from wrapping.
# install_path: Destination to install the library.
# module_template: The template file (.tpl) from which to generate the Pybind11 module.
# libs: Libraries to link with.
# dependencies: Dependencies which need to be built before the wrapper.
# use_boost (optional): Flag indicating whether to include Boost.
function(pybind_wrap
         target
         interface_header
         generated_cpp
         module_name
         top_namespace
         ignore_classes
         module_template
         libs
         dependencies)
  set(ExtraMacroArgs ${ARGN})
  list(GET ExtraMacroArgs 0 USE_BOOST)
  if(USE_BOOST)
    set(_WRAP_BOOST_ARG "--use-boost")
  else(USE_BOOST)
    set(_WRAP_BOOST_ARG "")
  endif(USE_BOOST)
  
  add_custom_command(OUTPUT ${generated_cpp}
                     COMMAND ${PYTHON_EXECUTABLE}
                             ${CMAKE_SOURCE_DIR}/wrap/pybind_wrapper.py
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
                             --template
                             ${module_template}
                             ${_WRAP_BOOST_ARG}
                     VERBATIM)
  add_custom_target(pybind_wrap_${module_name} ALL DEPENDS ${generated_cpp})

  # Late dependency injection, to make sure this gets called whenever the
  # interface header or the wrap library are updated.
  # ~~~
  # See: https://stackoverflow.com/questions/40032593/cmake-does-not-rebuild-dependent-after-prerequisite-changes
  # ~~~
  add_custom_command(OUTPUT ${generated_cpp}
                     DEPENDS ${interface_header}
                             ${CMAKE_SOURCE_DIR}/wrap/interface_parser.py
                             ${CMAKE_SOURCE_DIR}/wrap/pybind_wrapper.py
                             ${CMAKE_SOURCE_DIR}/wrap/template_instantiator.py
                     APPEND)

  pybind11_add_module(${target} ${generated_cpp})

  if(APPLE)
    # `type_info` objects will become "weak private external" if the templated class is initialized implicitly even if we explicitly
    # export them with `WRAP_EXPORT`. If that happens, the `type_info` for the same templated class will diverge between shared
    # libraries, causing `dynamic_cast` to fail. This is mitigated by telling Clang to mimic the MSVC behavior.
    # See https://developer.apple.com/library/archive/technotes/tn2185/_index.html#//apple_ref/doc/uid/DTS10004200-CH1-SUBSECTION2
    # https://github.com/CppMicroServices/CppMicroServices/pull/82/files
    # https://www.russellmcc.com/posts/2013-08-03-rtti.html
    target_compile_options(${target} PRIVATE "-fvisibility-ms-compat")
  endif()
  add_dependencies(${target} pybind_wrap_${module_name})
  if(NOT "${libs}" STREQUAL "")
    target_link_libraries(${target} PRIVATE "${libs}")
  endif()
  if(NOT "${dependencies}" STREQUAL "")
    add_dependencies(${target} ${dependencies})
  endif()
endfunction()

# Helper function to install python scripts and handle multiple build types
# where the scripts should be installed to all build type toolboxes
#
# Arguments:
# ~~~
# source_directory: The source directory to be installed. "The last component
#     of each directory name is appended to the destination directory but a
#     trailing slash may be used to avoid this because it leaves the last
#     component empty."
#     (https://cmake.org/cmake/help/v3.3/command/install.html?highlight=install#installing-directories)
# dest_directory: The destination directory to install to.
# patterns: list of file patterns to install
# ~~~
function(install_python_scripts
         source_directory
         dest_directory
         patterns)
  set(patterns_args "")
  set(exclude_patterns "")

  foreach(pattern ${patterns})
    list(APPEND patterns_args PATTERN "${pattern}")
  endforeach()
  if(WRAP_BUILD_TYPE_POSTFIXES)
    foreach(build_type ${CMAKE_CONFIGURATION_TYPES})
      string(TOUPPER "${build_type}" build_type_upper)
      if(${build_type_upper} STREQUAL "RELEASE")
        set(build_type_tag "") # Don't create release mode tag on installed
                               # directory
      else()
        set(build_type_tag "")
      endif()
      # Split up filename to strip trailing '/' in WRAP_CYTHON_INSTALL_PATH if
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

# Helper function to install specific files and handle multiple build types
# where the scripts should be installed to all build type toolboxes
#
# Arguments:
# ~~~
# source_files: The source files to be installed.
# dest_directory: The destination directory to install to.
function(install_python_files source_files dest_directory)

  if(WRAP_BUILD_TYPE_POSTFIXES)
    foreach(build_type ${CMAKE_CONFIGURATION_TYPES})
      string(TOUPPER "${build_type}" build_type_upper)
      set(build_type_tag "")
      # Split up filename to strip trailing '/' in WRAP_PY_INSTALL_PATH if
      # there is one
      get_filename_component(location "${dest_directory}" PATH)
      get_filename_component(name "${dest_directory}" NAME)
      install(FILES "${source_files}"
              DESTINATION "${location}/${name}${build_type_tag}"
              CONFIGURATIONS "${build_type}")
    endforeach()
  else()
    install(FILES "${source_files}" DESTINATION "${dest_directory}")
  endif()

endfunction()

# ~~~
# https://stackoverflow.com/questions/13959434/cmake-out-of-source-build-python-files
# ~~~
function(create_symlinks source_folder dest_folder)
  if(${source_folder} STREQUAL ${dest_folder})
    return()
  endif()

  file(GLOB files
       LIST_DIRECTORIES true
       RELATIVE "${source_folder}"
       "${source_folder}/*")
  foreach(path_file ${files})
    get_filename_component(folder ${path_file} PATH)
    get_filename_component(ext ${path_file} EXT)
    set(ignored_ext ".tpl" ".h")
    list (FIND ignored_ext "${ext}" _index)
    if (${_index} GREATER -1)
      continue()
    endif ()
    # Create REAL folder
    file(MAKE_DIRECTORY "${dest_folder}")

    # Delete symlink if it exists
    file(REMOVE "${dest_folder}/${path_file}")

    # Get OS dependent path to use in `execute_process`
    file(TO_NATIVE_PATH "${dest_folder}/${path_file}" link)
    file(TO_NATIVE_PATH "${source_folder}/${path_file}" target)

    # cmake-format: off
    if(UNIX)
      set(command ln -s ${target} ${link})
    else()
      set(command cmd.exe /c mklink ${link} ${target})
    endif()
    # cmake-format: on

    execute_process(COMMAND ${command}
                    RESULT_VARIABLE result
                    ERROR_VARIABLE output)

    if(NOT ${result} EQUAL 0)
      message(
        FATAL_ERROR
          "Could not create symbolic link for: ${target} --> ${output}")
    endif()

  endforeach(path_file)
endfunction(create_symlinks)
