#!/bin/bash

##########################################################
# Build and test the GTSAM Python wrapper.
##########################################################

set -x -e

# install TBB with _debug.so files
function install_tbb()
{
  echo install_tbb
  if [ "$(uname)" == "Linux" ]; then
    sudo apt-get -y install libtbb-dev

  elif [ "$(uname)" == "Darwin" ]; then
    brew install tbb
  fi
}

if [ -z ${PYTHON_VERSION+x} ]; then
    echo "Please provide the Python version to build against!"
    exit 127
fi

export PYTHON="python${PYTHON_VERSION}"

function install_dependencies()
{
  if [[ $(uname) == "Darwin" ]]; then
    brew install wget
  else
    # Install a system package required by our library
    sudo apt-get install -y wget libicu-dev python3-pip python3-setuptools
  fi

  export PATH=$PATH:$($PYTHON -c "import site; print(site.USER_BASE)")/bin

  if [ "${GTSAM_WITH_TBB:-OFF}" == "ON" ]; then
    install_tbb
  fi

  $PYTHON -m pip install -r $GITHUB_WORKSPACE/python/requirements.txt
}

function build()
{
  export CMAKE_GENERATOR=Ninja
  BUILD_PYBIND="ON"
  cmake $GITHUB_WORKSPACE \
      -B build \
      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} \
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
      -DGTSAM_ALLOW_DEPRECATED_SINCE_V43=OFF \
      -DCMAKE_INSTALL_PREFIX=$GITHUB_WORKSPACE/gtsam_install


  # Set to 2 cores so that Actions does not error out during resource provisioning.
  cmake --build build -j2

  $PYTHON -m pip install --user build/python
}

function test()
{
  cd $GITHUB_WORKSPACE/python/gtsam/tests
  $PYTHON -m unittest discover -v
  cd $GITHUB_WORKSPACE
}

# select between build or test
case $1 in
  -d)
    install_dependencies
    ;;
  -b)
    build
    ;;
  -t)
    test
    ;;
esac
