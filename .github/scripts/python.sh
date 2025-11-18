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
}

function build()
{
  export CMAKE_GENERATOR=Ninja
  BUILD_PYBIND="ON"

  # Add Boost hints on Windows
  BOOST_CMAKE_ARGS=""
  if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "win32" || "$OSTYPE" == "cygwin" ]]; then
    if [ -n "${BOOST_ROOT}" ]; then
     BOOST_ROOT_UNIX=$(echo "$BOOST_ROOT" | sed 's/\\/\//g')
     BOOST_CMAKE_ARGS="-DBOOST_ROOT=${BOOST_ROOT_UNIX} -DBOOST_INCLUDEDIR=${BOOST_ROOT_UNIX}/include -DBOOST_LIBRARYDIR=${BOOST_ROOT_UNIX}/lib"
    fi
  fi

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
      -DGTSAM_USE_BOOST_FEATURES=ON \
      -DGTSAM_ENABLE_BOOST_SERIALIZATION=ON \
      -DGTSAM_ALLOW_DEPRECATED_SINCE_V43=OFF \
      -DCMAKE_INSTALL_PREFIX=$GITHUB_WORKSPACE/gtsam_install \
      $BOOST_CMAKE_ARGS

  # Set to 2 cores so that Actions does not error out during resource provisioning.
  cmake --build build -j2

  cmake --build build --target python-install
}

function test()
{
  cd $GITHUB_WORKSPACE/python/gtsam/tests
  $PYTHON -m unittest discover -v
  cd $GITHUB_WORKSPACE

  cd $GITHUB_WORKSPACE/python/gtsam_unstable/tests
  $PYTHON -m unittest discover -v
  cd $GITHUB_WORKSPACE

  # cmake --build build --target python-test
  # cmake --build build --target python-test-unstable
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
