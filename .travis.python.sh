#!/bin/bash
set -x -e

if [ -z ${PYTHON_VERSION+x} ]; then
    echo "Please provide the Python version to build against!"
    exit 127
fi

PYTHON="python${PYTHON_VERSION}"

if [[ $(uname) == "Darwin" ]]; then
    brew install wget
else
    # Install a system package required by our library
    sudo apt-get install wget libicu-dev python3-pip python3-setuptools
fi

CURRDIR=$(pwd)

sudo $PYTHON -m pip install -r ./cython/requirements.txt

mkdir $CURRDIR/build
cd $CURRDIR/build

cmake $CURRDIR -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_UNSTABLE=ON \
    -DGTSAM_USE_QUATERNIONS=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_INSTALL_CYTHON_TOOLBOX=ON \
    -DGTSAM_PYTHON_VERSION=$PYTHON_VERSION \
    -DGTSAM_ALLOW_DEPRECATED_SINCE_V4=OFF \
    -DCMAKE_INSTALL_PREFIX=$CURRDIR/../gtsam_install

make -j$(nproc) install

cd cython

sudo $PYTHON setup.py install

cd $CURRDIR/cython/gtsam/tests

$PYTHON -m unittest discover