#!/bin/bash

##########################################################
# Build and test the GTSAM Python wrapper.
##########################################################

set -x -e

# install TBB with _debug.so files
function install_tbb()
{
  TBB_BASEURL=https://github.com/oneapi-src/oneTBB/releases/download
  TBB_VERSION=4.4.5
  TBB_DIR=tbb44_20160526oss
  TBB_SAVEPATH="/tmp/tbb.tgz"

  if [ "$(uname)" == "Linux" ]; then
    OS_SHORT="lin"
    TBB_LIB_DIR="intel64/gcc4.4"
    SUDO="sudo"

  elif [ "$(uname)" == "Darwin" ]; then
    OS_SHORT="osx"
    TBB_LIB_DIR=""
    SUDO=""

  fi

  wget "${TBB_BASEURL}/${TBB_VERSION}/${TBB_DIR}_${OS_SHORT}.tgz" -O $TBB_SAVEPATH
  tar -C /tmp -xf $TBB_SAVEPATH

  TBBROOT=/tmp/$TBB_DIR
  # Copy the needed files to the correct places.
  # This works correctly for CI builds, instead of setting path variables.
  # This is what Homebrew does to install TBB on Macs
  $SUDO cp -R $TBBROOT/lib/$TBB_LIB_DIR/* /usr/local/lib/
  $SUDO cp -R $TBBROOT/include/ /usr/local/include/

}

if [ -z ${PYTHON_VERSION+x} ]; then
    echo "Please provide the Python version to build against!"
    exit 127
fi

PYTHON="python${PYTHON_VERSION}"

if [[ $(uname) == "Darwin" ]]; then
    brew install wget
else
    # Install a system package required by our library
    sudo apt-get install -y wget libicu-dev python3-pip python3-setuptools
fi

PATH=$PATH:$($PYTHON -c "import site; print(site.USER_BASE)")/bin

[ "${GTSAM_WITH_TBB:-OFF}" = "ON" ] && install_tbb


BUILD_PYBIND="ON"

sudo $PYTHON -m pip install -r $GITHUB_WORKSPACE/python/requirements.txt

mkdir $GITHUB_WORKSPACE/build
cd $GITHUB_WORKSPACE/build

cmake $GITHUB_WORKSPACE -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_UNSTABLE=${GTSAM_BUILD_UNSTABLE:-ON} \
    -DGTSAM_USE_QUATERNIONS=OFF \
    -DGTSAM_WITH_TBB=${GTSAM_WITH_TBB:-OFF} \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_BUILD_PYTHON=${BUILD_PYBIND} \
    -DGTSAM_UNSTABLE_BUILD_PYTHON=${GTSAM_BUILD_UNSTABLE:-ON} \
    -DGTSAM_PYTHON_VERSION=$PYTHON_VERSION \
    -DPYTHON_EXECUTABLE:FILEPATH=$(which $PYTHON) \
    -DGTSAM_ALLOW_DEPRECATED_SINCE_V41=OFF \
    -DCMAKE_INSTALL_PREFIX=$GITHUB_WORKSPACE/gtsam_install


# Set to 2 cores so that Actions does not error out during resource provisioning.
make -j2 install

cd $GITHUB_WORKSPACE/build/python
$PYTHON setup.py install --user --prefix=
cd $GITHUB_WORKSPACE/python/gtsam/tests
$PYTHON -m unittest discover -v
