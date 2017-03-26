# Install cython components
include(GtsamCythonWrap)
# install scripts and tests
install_cython_scripts("${CMAKE_SOURCE_DIR}/cython/gtsam" "${GTSAM_CYTHON_INSTALL_PATH}/gtsam" "*.py")
