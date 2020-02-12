#!/bin/bash
set -x -e

# Install a system package required by our library
sudo apt-get install wget libicu-dev

CURRDIR=$(pwd)

sudo python -m pip install -r ./cython/requirements.txt

mkdir $CURRDIR/build
cd $CURRDIR/build

cmake $CURRDIR -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_UNSTABLE=ON \
    -DGTSAM_USE_QUATERNIONS=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_INSTALL_CYTHON_TOOLBOX=ON \
    -DGTSAM_PYTHON_VERSION=Default \
    -DGTSAM_ALLOW_DEPRECATED_SINCE_V4=OFF \
    -DCMAKE_INSTALL_PREFIX=$CURRDIR/../gtsam_install

make -j$(nproc) install

cd $CURRDIR/../gtsam_install/cython

sudo python setup.py install

cd $CURRDIR/cython/gtsam/tests

python -m unittest discover