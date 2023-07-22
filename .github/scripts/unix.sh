#!/bin/bash

##########################################################
# Build and test GTSAM for *nix based systems.
# Specifically Linux and macOS.
##########################################################

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

# common tasks before either build or test
function configure()
{
  # delete old build
  rm -rf build

  if [ "${GTSAM_WITH_TBB:-OFF}" == "ON" ]; then
    install_tbb
  fi

  if [ ! -z "$GCC_VERSION" ]; then
    export CC=gcc-$GCC_VERSION
    export CXX=g++-$GCC_VERSION
  fi

  # GTSAM_BUILD_WITH_MARCH_NATIVE=OFF: to avoid crashes in builder VMs
  # CMAKE_CXX_FLAGS="-w": Suppress warnings to avoid IO latency in CI logs
  export CMAKE_GENERATOR=Ninja
  cmake . \
      -B build \
      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE:-Debug} \
      -DCMAKE_CXX_FLAGS="-w" \
      -DGTSAM_BUILD_TESTS=${GTSAM_BUILD_TESTS:-OFF} \
      -DGTSAM_BUILD_UNSTABLE=${GTSAM_BUILD_UNSTABLE:-ON} \
      -DGTSAM_WITH_TBB=${GTSAM_WITH_TBB:-OFF} \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=${GTSAM_BUILD_EXAMPLES_ALWAYS:-ON} \
      -DGTSAM_ALLOW_DEPRECATED_SINCE_V43=${GTSAM_ALLOW_DEPRECATED_SINCE_V43:-OFF} \
      -DGTSAM_USE_QUATERNIONS=${GTSAM_USE_QUATERNIONS:-OFF} \
      -DGTSAM_ROT3_EXPMAP=${GTSAM_ROT3_EXPMAP:-ON} \
      -DGTSAM_POSE3_EXPMAP=${GTSAM_POSE3_EXPMAP:-ON} \
      -DGTSAM_USE_SYSTEM_EIGEN=${GTSAM_USE_SYSTEM_EIGEN:-OFF} \
      -DGTSAM_USE_SYSTEM_METIS=${GTSAM_USE_SYSTEM_METIS:-OFF} \
      -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
      -DGTSAM_SINGLE_TEST_EXE=OFF
}


# common tasks after either build or test
function finish ()
{
  # Print ccache stats
  if [ -x "$(command -v ccache)" ]; then
    ccache -s
  fi
}

# compile the code with the intent of populating the cache
function build ()
{
  export GTSAM_BUILD_EXAMPLES_ALWAYS=ON
  export GTSAM_BUILD_TESTS=OFF

  configure

  if [ "$(uname)" == "Linux" ]; then
    if (($(nproc) > 2)); then
      cmake --build build -j4
    else
      cmake --build build -j2
    fi
  elif [ "$(uname)" == "Darwin" ]; then
    cmake --build build -j$(sysctl -n hw.physicalcpu)
  fi

  finish
}

# run the tests
function test ()
{
  export GTSAM_BUILD_EXAMPLES_ALWAYS=OFF
  export GTSAM_BUILD_TESTS=ON

  configure

  # Actual testing
  if [ "$(uname)" == "Linux" ]; then
    if (($(nproc) > 2)); then
      cmake --build build --config Release -j$(nproc) --target check
    else
      cmake --build build --config Release -j2 --target check
    fi
  elif [ "$(uname)" == "Darwin" ]; then
    cmake --build build --config Release -j$(sysctl -n hw.physicalcpu) --target check
  fi

  finish
}

set -e   # Make sure any error makes the script to return an error code
set -x   # echo

# select between build or test
case $1 in
  -b)
    build
    ;;
  -t)
    test
    ;;
esac
