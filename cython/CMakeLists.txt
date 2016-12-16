# Install cython components
include(GtsamCythonWrap)
# install scripts and tests
install_cython_scripts("${CMAKE_SOURCE_DIR}/cython/gtsam" "*.py")
