# Create directory where cython build files will be placed
file(MAKE_DIRECTORY ${GTSAM_CYTHON_INSTALL_PATH})

# Install cython components
include(GtsamCythonWrap)

# Create the cython toolbox for the gtsam library
if (GTSAM_INSTALL_CYTHON_TOOLBOX)
  # build and include the eigency version of eigency
  add_subdirectory(gtsam_eigency)
  include_directories(${GTSAM_EIGENCY_PATH})

  # Fix for error "C1128: number of sections exceeded object file format limit"
  if(MSVC)
    add_compile_options(/bigobj)
  endif()

  # wrap gtsam
  add_custom_target(gtsam_header DEPENDS "../gtsam.h")
  wrap_and_install_library_cython("../gtsam.h" # interface_header
                                  ""                  # extra imports
                                  "${GTSAM_CYTHON_INSTALL_PATH}/gtsam" # install path
                                  gtsam  # library to link with
                                  "wrap;cythonize_eigency;gtsam;gtsam_header"  # dependencies which need to be built before wrapping
                                  )

  # wrap gtsam_unstable
  if(GTSAM_BUILD_UNSTABLE)
    add_custom_target(gtsam_unstable_header DEPENDS "../gtsam_unstable/gtsam_unstable.h")
    wrap_and_install_library_cython("../gtsam_unstable/gtsam_unstable.h" # interface_header
                                    "from gtsam.gtsam cimport *"                  # extra imports
                                    "${GTSAM_CYTHON_INSTALL_PATH}/gtsam_unstable" # install path
                                    gtsam_unstable  # library to link with
                                    "gtsam_unstable;gtsam_unstable_header;cythonize_gtsam"  # dependencies to be built before wrapping
                                    )
  endif()

  set(CYTHON_INSTALL_REQUIREMENTS_FILE "${PROJECT_SOURCE_DIR}/cython/requirements.txt")

  # Install the custom-generated __init__.py
  # This is to make the build/cython/gtsam folder a python package, so gtsam can be found while wrapping gtsam_unstable
  configure_file(${PROJECT_SOURCE_DIR}/cython/gtsam/__init__.py ${PROJECT_BINARY_DIR}/cython/gtsam/__init__.py COPYONLY)
  configure_file(${PROJECT_SOURCE_DIR}/cython/gtsam_unstable/__init__.py ${PROJECT_BINARY_DIR}/cython/gtsam_unstable/__init__.py COPYONLY)
  configure_file(${PROJECT_SOURCE_DIR}/cython/setup.py.in ${PROJECT_BINARY_DIR}/cython/setup.py)
  install_cython_files("${PROJECT_BINARY_DIR}/cython/setup.py" "${GTSAM_CYTHON_INSTALL_PATH}")
  # install scripts and tests
  install_cython_scripts("${PROJECT_SOURCE_DIR}/cython/gtsam" "${GTSAM_CYTHON_INSTALL_PATH}" "*.py")
  install_cython_scripts("${PROJECT_SOURCE_DIR}/cython/gtsam_unstable" "${GTSAM_CYTHON_INSTALL_PATH}" "*.py")

  # Install gtsam_eigency.
  # The paths are picked up directly from the parent CMakeLists.txt.
  install_gtsam_eigency()

  # Automatically run the python installation via the setup.py
  install(CODE
    "execute_process(COMMAND ${PYTHON_EXECUTABLE} setup.py install
      WORKING_DIRECTORY ${GTSAM_CYTHON_INSTALL_PATH})")

endif ()
