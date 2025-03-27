#!/bin/bash

# This script is invoked prior to building the wheels with cibuildwheel. It is used in the build-cibw.yml workflow in .github/workflows.
# It installs the necessary dependencies and builds the wrapper module for the specified Python version.

set -e
set -x

PYTHON_VERSION="$1"
PROJECT_DIR="$2"
ARCH=$(uname -m)

export PYTHON="python${PYTHON_VERSION}"

if [ "$(uname)" == "Linux" ]; then
    # manylinux2014 is based on CentOS 7, so use yum to install dependencies
    yum install -y wget doxygen

    # Install Boost from source
    wget https://archives.boost.io/release/1.87.0/source/boost_1_87_0.tar.gz --quiet
    tar -xzf boost_1_87_0.tar.gz
    cd boost_1_87_0
    ./bootstrap.sh --prefix=/opt/boost
    ./b2 install --prefix=/opt/boost --with=all -d0
    cd ..
elif [ "$(uname)" == "Darwin" ]; then
    brew install wget cmake boost
    export CC=clang
    export CXX=clang++
fi

$(which $PYTHON) -m pip install -r $PROJECT_DIR/python/dev_requirements.txt

# Remove build/cache files that were generated on host
rm -rf $PROJECT_DIR/build
rm -rf CMakeCache.txt CMakeFiles

# Build the Python wrapper module
cmake $PROJECT_DIR \
    -B build \
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_UNSTABLE=${GTSAM_BUILD_UNSTABLE:-ON} \
    -DGTSAM_USE_QUATERNIONS=OFF \
    -DGTSAM_WITH_TBB=${GTSAM_WITH_TBB:-OFF} \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_BUILD_PYTHON=ON \
    -DGTSAM_UNSTABLE_BUILD_PYTHON=${GTSAM_BUILD_UNSTABLE:-ON} \
    -DGTSAM_PYTHON_VERSION=$PYTHON_VERSION \
    -DPYTHON_EXECUTABLE:FILEPATH=$(which $PYTHON) \
    -DGTSAM_ALLOW_DEPRECATED_SINCE_V43=OFF \
    -DCMAKE_INSTALL_PREFIX=$PROJECT_DIR/gtsam_install \
    -DGTSAM_GENERATE_DOC_XML=0

# Generate Doxygen XML documentation
doxygen build/doc/Doxyfile

# Install the Python wrapper module and generate Python stubs
cd $PROJECT_DIR/build/python
if [ "$(uname)" == "Linux" ]; then
    make -j $(nproc) install
    make -j $(nproc) python-stubs
elif [ "$(uname)" == "Darwin" ]; then
    make -j $(sysctl -n hw.logicalcpu) install
    make -j $(sysctl -n hw.logicalcpu) python-stubs
fi

