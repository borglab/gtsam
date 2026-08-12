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
NO_BOOST_BUILD=OFF
SCCACHE=OFF
BUILD_JOBS=${BUILD_JOBS:-2}

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
  BUILD_PYBIND="ON"
  USE_BOOST_FEATURES="ON"
  ENABLE_BOOST_SERIALIZATION="ON"

  if [ "${NO_BOOST_BUILD}" == "ON" ]; then
    USE_BOOST_FEATURES="OFF"
    ENABLE_BOOST_SERIALIZATION="OFF"
  fi

  # Add Boost hints on Windows
  BOOST_CMAKE_ARGS=""
  if [ "${NO_BOOST_BUILD}" != "ON" ] && [[ "$OSTYPE" == "msys" || "$OSTYPE" == "win32" || "$OSTYPE" == "cygwin" ]]; then
    if [ -n "${BOOST_ROOT}" ]; then
      BOOST_ROOT_UNIX=$(echo "$BOOST_ROOT" | sed 's/\\/\//g')
      BOOST_CMAKE_ARGS="-DBOOST_ROOT=${BOOST_ROOT_UNIX}"
      if [ -n "${BOOST_INCLUDEDIR}" ]; then
        BOOST_INCLUDEDIR_UNIX=$(echo "$BOOST_INCLUDEDIR" | sed 's/\\/\//g')
        BOOST_CMAKE_ARGS="${BOOST_CMAKE_ARGS} -DBOOST_INCLUDEDIR=${BOOST_INCLUDEDIR_UNIX}"
      fi
      if [ -n "${BOOST_LIBRARYDIR}" ]; then
        BOOST_LIBRARYDIR_UNIX=$(echo "$BOOST_LIBRARYDIR" | sed 's/\\/\//g')
        BOOST_CMAKE_ARGS="${BOOST_CMAKE_ARGS} -DBOOST_LIBRARYDIR=${BOOST_LIBRARYDIR_UNIX}"
      fi
    fi
  fi

  SCCACHE_CMAKE_ARGS=""
  if [ "${SCCACHE}" == "ON" ]; then
    SCCACHE_CMAKE_ARGS="-DCMAKE_C_COMPILER_LAUNCHER=sccache -DCMAKE_CXX_COMPILER_LAUNCHER=sccache"
  fi

  cmake $GITHUB_WORKSPACE \
      -B build -G Ninja \
      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} \
      -DGTSAM_BUILD_TESTS=${GTSAM_BUILD_TESTS:-OFF} \
      -DGTSAM_SLOW_BUT_CORRECT_BETWEENFACTOR=ON \
      -DGTSAM_BUILD_UNSTABLE=${GTSAM_BUILD_UNSTABLE:-ON} \
      -DGTSAM_USE_QUATERNIONS=OFF \
      -DGTSAM_WITH_TBB=${GTSAM_WITH_TBB:-OFF} \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
      -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
      -DGTSAM_BUILD_WITH_PRECOMPILED_HEADERS=OFF \
      -DGTSAM_BUILD_PYTHON=${BUILD_PYBIND} \
      -DGTSAM_UNSTABLE_BUILD_PYTHON=${GTSAM_BUILD_UNSTABLE:-ON} \
      -DGTSAM_PYTHON_VERSION=$PYTHON_VERSION \
      -DPYTHON_EXECUTABLE:FILEPATH=$(which $PYTHON) \
      -DGTSAM_USE_BOOST_FEATURES=${USE_BOOST_FEATURES} \
      -DGTSAM_ENABLE_BOOST_SERIALIZATION=${ENABLE_BOOST_SERIALIZATION} \
      -DGTSAM_ALLOW_DEPRECATED_SINCE_V43=OFF \
      -DCMAKE_INSTALL_PREFIX=$GITHUB_WORKSPACE/gtsam_install \
      $BOOST_CMAKE_ARGS \
      $SCCACHE_CMAKE_ARGS

  # Unset environment variables so sccache can reuse caches.
  unset PYTHON
  unset PYTHON_VERSION
  if [ "${NO_BOOST_BUILD}" == "ON" ]; then
    # Build only wrapper targets for the no-Boost verification lane.
    cmake --build build -j"${BUILD_JOBS}" --target gtsam_py gtsam_unstable_py
  else
    # Limit parallelism so that Actions does not run out of resources.
    cmake --build build -j"${BUILD_JOBS}"
    cmake --build build --target python-install
  fi
}

function test()
{
  if [ "${NO_BOOST_BUILD}" == "ON" ]; then
    if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "win32" || "$OSTYPE" == "cygwin" ]]; then
      PYTHON_BUILD_DIRECTORY=$(cygpath -w "$GITHUB_WORKSPACE/build/python")
      export PYTHONPATH="${PYTHON_BUILD_DIRECTORY}${PYTHONPATH:+;$PYTHONPATH}"
    else
      export PYTHONPATH="$GITHUB_WORKSPACE/build/python${PYTHONPATH:+:$PYTHONPATH}"
    fi
  fi

  cd $GITHUB_WORKSPACE/python/gtsam/tests
  $PYTHON -m unittest discover -v
  cd $GITHUB_WORKSPACE

  cd $GITHUB_WORKSPACE/python/gtsam_unstable/tests
  $PYTHON -m unittest discover -v
  cd $GITHUB_WORKSPACE

  # cmake --build build --target python-test
  # cmake --build build --target python-test-unstable
}

# Parse optional flags.
if [ $# -lt 1 ]; then
  echo "Usage: $0 {-d|-b|-t} [--no-boost]"
  exit 2
fi

ACTION="$1"
shift

while [ $# -gt 0 ]; do
  case "$1" in
    --no-boost)
      NO_BOOST_BUILD=ON
      ;;
    --sccache)
      SCCACHE=ON
      ;;
    *)
      echo "Unknown option: $1"
      exit 2
      ;;
  esac
  shift
done

# select between build or test
case $ACTION in
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
