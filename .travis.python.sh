#!/bin/bash
set -x -e

if [ -z ${PYTHON_VERSION+x} ]; then
    echo "Please provide the Python version to build against!"
    exit 127
fi

if [ -z ${WRAPPER+x} ]; then
    echo "Please provide the wrapper to build!"
    exit 126
fi

PYTHON="python${PYTHON_VERSION}"

if [[ $(uname) == "Darwin" ]]; then
    brew install wget
else
    # Install a system package required by our library
    sudo apt-get install wget libicu-dev python3-pip python3-setuptools
fi

PATH=$PATH:$($PYTHON -c "import site; print(site.USER_BASE)")/bin

case $WRAPPER in
"cython")
    BUILD_CYTHON="ON"
    BUILD_PYBIND="OFF"
    TYPEDEF_POINTS_TO_VECTORS="OFF"

    $PYTHON -m pip install --user -r ./cython/requirements.txt
    ;;
"pybind")
    BUILD_CYTHON="OFF"
    BUILD_PYBIND="ON"
    TYPEDEF_POINTS_TO_VECTORS="ON"

    $PYTHON -m pip install --user -r ./wrap/python/requirements.txt
    ;;
*)
    exit 126
    ;;
esac

CURRDIR=$(pwd)

mkdir $CURRDIR/build
cd $CURRDIR/build

cmake $CURRDIR -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_UNSTABLE=ON \
    -DGTSAM_USE_QUATERNIONS=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_INSTALL_CYTHON_TOOLBOX=${BUILD_CYTHON} \
    -DGTSAM_BUILD_PYTHON=${BUILD_PYBIND} \
    -DGTSAM_TYPEDEF_POINTS_TO_VECTORS=${TYPEDEF_POINTS_TO_VECTORS} \
    -DGTSAM_PYTHON_VERSION=$PYTHON_VERSION \
    -DGTSAM_ALLOW_DEPRECATED_SINCE_V4=OFF \
    -DPYTHON_EXECUTABLE:FILEPATH=$(which $PYTHON) \
    -DCMAKE_INSTALL_PREFIX=$CURRDIR/../gtsam_install

make -j$(nproc) install

case $WRAPPER in
"cython")
    cd $CURRDIR/../gtsam_install/cython
    $PYTHON setup.py install --user --prefix=
    cd $CURRDIR/cython/gtsam/tests
    $PYTHON -m unittest discover
    ;;
"pybind")
    $PYTHON setup.py install --user --prefix=
    cd $CURRDIR/wrap/python/gtsam_py/tests
    $PYTHON -m unittest discover
    ;;
*)
    echo "THIS SHOULD NEVER HAPPEN!"
    exit 125
    ;;
esac
