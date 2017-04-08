# Install cython components
include(GtsamCythonWrap)
# install scripts and tests
install_cython_scripts("${PROJECT_SOURCE_DIR}/cython/gtsam" "${GTSAM_CYTHON_INSTALL_PATH}" "*.py")


# generate __init__.py into build folder (configured with or without gtsam_unstable import line)
# This also makes the build/cython/gtsam folder a python package, so gtsam can be found while wrapping gtsam_unstable
if(GTSAM_BUILD_UNSTABLE)
   set(GTSAM_UNSTABLE_IMPORT "from gtsam_unstable import *")
endif()
configure_file(${PROJECT_SOURCE_DIR}/cython/gtsam/__init__.py.in ${PROJECT_BINARY_DIR}/cython/gtsam/__init__.py)

# Install the custom-generated __init__.py
install_cython_files("${PROJECT_BINARY_DIR}/cython/gtsam/__init__.py" "${GTSAM_CYTHON_INSTALL_PATH}/gtsam")
