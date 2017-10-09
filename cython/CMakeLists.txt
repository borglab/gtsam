# Install cython components
include(GtsamCythonWrap)

# Create the cython toolbox for the gtsam library
if (GTSAM_INSTALL_CYTHON_TOOLBOX)
  # build and include the eigency version of eigency
  add_subdirectory(gtsam_eigency)
  include_directories(${PROJECT_BINARY_DIR}/cython/gtsam_eigency)

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
    set(GTSAM_UNSTABLE_IMPORT "from gtsam_unstable import *")
    wrap_and_install_library_cython("../gtsam_unstable/gtsam_unstable.h" # interface_header
                                    "from gtsam.gtsam cimport *"                  # extra imports
                                    "${GTSAM_CYTHON_INSTALL_PATH}/gtsam" # install path
                                    gtsam_unstable  # library to link with
                                    "gtsam_unstable;gtsam_unstable_header;cythonize_gtsam"  # dependencies to be built before wrapping
                                    )
    # for some reasons cython gtsam_unstable can't find gtsam/gtsam.pxd without doing this
    file(WRITE ${PROJECT_BINARY_DIR}/cython/gtsam_unstable/__init__.py "")
  endif()

  # Install the custom-generated __init__.py
  # This is to make the build/cython/gtsam folder a python package, so gtsam can be found while wrapping gtsam_unstable
  configure_file(${PROJECT_SOURCE_DIR}/cython/gtsam/__init__.py.in ${PROJECT_BINARY_DIR}/cython/gtsam/__init__.py)
  install_cython_files("${PROJECT_BINARY_DIR}/cython/gtsam/__init__.py" "${GTSAM_CYTHON_INSTALL_PATH}/gtsam")
  # install scripts and tests
  install_cython_scripts("${PROJECT_SOURCE_DIR}/cython/gtsam" "${GTSAM_CYTHON_INSTALL_PATH}" "*.py")

endif ()
