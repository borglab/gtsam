# Install cython components
include(GtsamCythonWrap)

# Create the cython toolbox for the gtsam library
if (GTSAM_INSTALL_CYTHON_TOOLBOX)
  # Add the new make target command
  set(python_install_target python-install)
  add_custom_target(${python_install_target}
    COMMAND ${PYTHON_EXECUTABLE} ${GTSAM_CYTHON_INSTALL_PATH}/setup.py install
    WORKING_DIRECTORY ${GTSAM_CYTHON_INSTALL_PATH})

  # build and include the eigency version of eigency
  add_subdirectory(gtsam_eigency)
  include_directories(${GTSAM_EIGENCY_INSTALL_PATH})

  # Fix for error "C1128: number of sections exceeded object file format limit"
  if(MSVC)
    add_compile_options(/bigobj)
  endif()

  # First set up all the package related files.
  # This also ensures the below wrap operations work correctly.
  set(CYTHON_INSTALL_REQUIREMENTS_FILE "${PROJECT_SOURCE_DIR}/cython/requirements.txt")

  # Install the custom-generated __init__.py
  # This makes the cython (sub-)directories into python packages, so gtsam can be found while wrapping gtsam_unstable
  configure_file(${PROJECT_SOURCE_DIR}/cython/gtsam/__init__.py ${GTSAM_CYTHON_INSTALL_PATH}/gtsam/__init__.py COPYONLY)
  configure_file(${PROJECT_SOURCE_DIR}/cython/gtsam_unstable/__init__.py ${GTSAM_CYTHON_INSTALL_PATH}/gtsam_unstable/__init__.py COPYONLY)
  configure_file(${PROJECT_SOURCE_DIR}/cython/setup.py.in ${GTSAM_CYTHON_INSTALL_PATH}/setup.py)

  # Wrap gtsam
  add_custom_target(gtsam_header DEPENDS "../gtsam.h")
  wrap_and_install_library_cython("../gtsam.h" # interface_header
                                  ""                  # extra imports
                                  "${GTSAM_CYTHON_INSTALL_PATH}/gtsam" # install path
                                  gtsam  # library to link with
                                  "wrap;cythonize_eigency;gtsam;gtsam_header"  # dependencies which need to be built before wrapping
                                  )
  add_dependencies(${python_install_target} gtsam gtsam_header)

  # Wrap gtsam_unstable
  if(GTSAM_BUILD_UNSTABLE)
    add_custom_target(gtsam_unstable_header DEPENDS "../gtsam_unstable/gtsam_unstable.h")
    wrap_and_install_library_cython("../gtsam_unstable/gtsam_unstable.h" # interface_header
                                    "from gtsam.gtsam cimport *"                  # extra imports
                                    "${GTSAM_CYTHON_INSTALL_PATH}/gtsam_unstable" # install path
                                    gtsam_unstable  # library to link with
                                    "gtsam_unstable;gtsam_unstable_header;cythonize_gtsam"  # dependencies to be built before wrapping
                                    )
    add_dependencies(${python_install_target} gtsam_unstable gtsam_unstable_header)
  endif()

  # install scripts and tests
  install_cython_scripts("${PROJECT_SOURCE_DIR}/cython/gtsam" "${GTSAM_CYTHON_INSTALL_PATH}" "*.py")
  install_cython_scripts("${PROJECT_SOURCE_DIR}/cython/gtsam_unstable" "${GTSAM_CYTHON_INSTALL_PATH}" "*.py")

endif ()
