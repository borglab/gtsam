# Unset these cached variables to avoid surprises when the python in the current
# environment are different from the cached!
unset(PYTHON_EXECUTABLE CACHE)
find_package(pybind11 REQUIRED)

# User-friendly Pybind11 wrapping and installing function. Builds a Pybind11
# module from the provided interface_header. For example, for the interface
# header gtsam.h, this will build the wrap module 'gtsam_py.cc'.
#
# Arguments:
#
# interface_header:  The relative path to the wrapper interface definition file.
# install_path: destination to install the library libs: libraries to link with
# dependencies: Dependencies which need to be built before the wrapper
function(pybind_wrapper
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
                             --src ${interface_header}
                             --out ${generated_cpp}
                             --module_name ${module_name}
                             --top_module_namespaces "${top_namespace}"
                             --ignore ${ignore_classes}
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
    target_link_libraries(${target} "${libs}")
  endif()
  if(NOT "${dependencies}" STREQUAL "")
    add_dependencies(${target} ${dependencies})
  endif()
endfunction()
