#!/bin/bash

##########################################################
# Build and test GTSAM for *nix based systems.
# Specifically Linux and macOS.
##########################################################

set -e   # Make sure any error makes the script to return an error code
set -x   # echo


# common tasks before either build or test
function configure()
{
  # delete old build
  rm -rf build

  # GTSAM_BUILD_WITH_MARCH_NATIVE=OFF: to avoid crashes in builder VMs
  # CMAKE_CXX_FLAGS="-w": Suppress warnings to avoid IO latency in CI logs
  export CMAKE_GENERATOR=Ninja
  cmake $GITHUB_WORKSPACE \
      -B build \
      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE:-Debug} \
      -DCMAKE_CXX_FLAGS="-w" \
      -DGTSAM_BUILD_TESTS=${GTSAM_BUILD_TESTS:-OFF} \
      -DGTSAM_BUILD_UNSTABLE=${GTSAM_BUILD_UNSTABLE:-ON} \
      -DGTSAM_WITH_TBB=${GTSAM_WITH_TBB:-OFF} \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=${GTSAM_BUILD_EXAMPLES_ALWAYS:-OFF} \
      -DGTSAM_ALLOW_DEPRECATED_SINCE_V43=${GTSAM_ALLOW_DEPRECATED_SINCE_V43:-OFF} \
      -DGTSAM_USE_QUATERNIONS=${GTSAM_USE_QUATERNIONS:-OFF} \
      -DGTSAM_ROT3_EXPMAP=${GTSAM_ROT3_EXPMAP:-ON} \
      -DGTSAM_POSE3_EXPMAP=${GTSAM_POSE3_EXPMAP:-ON} \
      -DGTSAM_USE_SYSTEM_EIGEN=${GTSAM_USE_SYSTEM_EIGEN:-OFF} \
      -DGTSAM_USE_SYSTEM_METIS=${GTSAM_USE_SYSTEM_METIS:-OFF} \
      -DGTSAM_USE_BOOST_FEATURES=${GTSAM_USE_BOOST_FEATURES:-OFF} \
      -DGTSAM_ENABLE_BOOST_SERIALIZATION=${GTSAM_ENABLE_BOOST_SERIALIZATION:-OFF} \
      -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
      -DGTSAM_SINGLE_TEST_EXE=${GTSAM_SINGLE_TEST_EXE:-OFF} \
      -DGTSAM_ENABLE_GEOGRAPHICLIB=${GTSAM_ENABLE_GEOGRAPHICLIB:-OFF}
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
  export GTSAM_BUILD_TESTS=ON

  configure

  # Actual testing
  if [ "$(uname)" == "Linux" ]; then
    if (($(nproc) > 2)); then
      cmake --build build -j$(nproc) --target check
    else
      cmake --build build -j2 --target check
    fi
  elif [ "$(uname)" == "Darwin" ]; then
    cmake --build build -j$(sysctl -n hw.physicalcpu) --target check
  fi

  finish
}

# select between build or test
case $1 in
  -b)
    build
    ;;
  -t)
    test
    ;;
esac
