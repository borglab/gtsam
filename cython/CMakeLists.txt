# Install cython components
include(GtsamCythonWrap)
# install scripts and tests
install_cython_scripts("${CMAKE_SOURCE_DIR}/cython/gtsam" "${GTSAM_CYTHON_INSTALL_PATH}" "*.py")
install_cython_scripts("${CMAKE_SOURCE_DIR}/cython/gtsam_unstable/tests" "${GTSAM_CYTHON_INSTALL_PATH}/gtsam/tests/" "*.py")
