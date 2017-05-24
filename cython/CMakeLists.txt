# Install cython components
include(GtsamCythonWrap)

# Create the cython toolbox for the gtsam library
if (GTSAM_INSTALL_CYTHON_TOOLBOX)
  wrap_and_install_library_cython("../gtsam.h" # interface_header
                                  ""                  # extra imports
                                  "${GTSAM_CYTHON_INSTALL_PATH}/gtsam" # install path
                                  gtsam  # dependencies which need to be built before the wrapper
                                  )

  # wrap gtsam_unstable
  if(GTSAM_BUILD_UNSTABLE)
    set(GTSAM_UNSTABLE_IMPORT "from gtsam_unstable import *")
    wrap_and_install_library_cython("../gtsam_unstable/gtsam_unstable.h" # interface_header
                                    "from gtsam.gtsam cimport *"                  # extra imports
                                    "${GTSAM_CYTHON_INSTALL_PATH}/gtsam" # install path
                                    gtsam_unstable  # dependencies which need to be built before the wrapper
                                    )
    add_dependencies(gtsam_unstable_cython_wrapper gtsam_cython_wrapper)
  endif()

  # Install the custom-generated __init__.py with gtsam_unstable disabled
  # This is to make the build/cython/gtsam folder a python package, so gtsam can be found while wrapping gtsam_unstable
  configure_file(${PROJECT_SOURCE_DIR}/cython/gtsam/__init__.py.in ${PROJECT_BINARY_DIR}/cython/gtsam/__init__.py)
  install_cython_files("${PROJECT_BINARY_DIR}/cython/gtsam/__init__.py" "${GTSAM_CYTHON_INSTALL_PATH}/gtsam")
  # install scripts and tests
  install_cython_scripts("${PROJECT_SOURCE_DIR}/cython/gtsam" "${GTSAM_CYTHON_INSTALL_PATH}" "*.py")

endif ()
