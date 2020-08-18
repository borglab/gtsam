#!/bin/bash

##########################################################
# Build and test the GTSAM Python wrapper.
##########################################################

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
    sudo apt-get install -y wget libicu-dev python3-pip python3-setuptools
fi

PATH=$PATH:$($PYTHON -c "import site; print(site.USER_BASE)")/bin

case $WRAPPER in
"cython")
    BUILD_CYTHON="ON"
    BUILD_PYBIND="OFF"
    TYPEDEF_POINTS_TO_VECTORS="OFF"

    sudo $PYTHON -m pip install -r $GITHUB_WORKSPACE/cython/requirements.txt
    ;;
"pybind")
    BUILD_CYTHON="OFF"
    BUILD_PYBIND="ON"
    TYPEDEF_POINTS_TO_VECTORS="ON"

    sudo $PYTHON -m pip install -r $GITHUB_WORKSPACE/python/requirements.txt
    ;;
*)
    exit 126
    ;;
esac

mkdir $GITHUB_WORKSPACE/build
cd $GITHUB_WORKSPACE/build

cmake $GITHUB_WORKSPACE -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_UNSTABLE=ON \
    -DGTSAM_USE_QUATERNIONS=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_INSTALL_CYTHON_TOOLBOX=${BUILD_CYTHON} \
    -DGTSAM_BUILD_PYTHON=${BUILD_PYBIND} \
    -DGTSAM_TYPEDEF_POINTS_TO_VECTORS=${TYPEDEF_POINTS_TO_VECTORS} \
    -DGTSAM_PYTHON_VERSION=$PYTHON_VERSION \
    -DPYTHON_EXECUTABLE:FILEPATH=$(which $PYTHON) \
    -DGTSAM_ALLOW_DEPRECATED_SINCE_V41=OFF \
    -DCMAKE_INSTALL_PREFIX=$GITHUB_WORKSPACE/gtsam_install

make -j$(nproc) install &

while ps -p $! > /dev/null
do
  sleep 60
  now=$(date +%s)
  printf "%d seconds have elapsed\n" $(( (now - start) ))
done

case $WRAPPER in
"cython")
    cd $GITHUB_WORKSPACE/build/cython
    $PYTHON setup.py install --user --prefix=
    cd $GITHUB_WORKSPACE/build/cython/gtsam/tests
    $PYTHON -m unittest discover
    ;;
"pybind")
    cd $GITHUB_WORKSPACE/build/python
    $PYTHON setup.py install --user --prefix=
    cd $GITHUB_WORKSPACE/python/gtsam/tests
    $PYTHON -m unittest discover
    ;;
*)
    echo "THIS SHOULD NEVER HAPPEN!"
    exit 125
    ;;
esac