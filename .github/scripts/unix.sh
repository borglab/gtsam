#!/bin/bash

##########################################################
# Build and test GTSAM for *nix based systems.
# Specifically Linux and macOS.
##########################################################

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

# common tasks before either build or test
function configure()
{
  set -e   # Make sure any error makes the script to return an error code
  set -x   # echo

  SOURCE_DIR=$GITHUB_WORKSPACE
  BUILD_DIR=$GITHUB_WORKSPACE/build

  #env
  rm -fr $BUILD_DIR || true
  mkdir $BUILD_DIR && cd $BUILD_DIR

  [ "${GTSAM_WITH_TBB:-OFF}" = "ON" ] && install_tbb

  if [ ! -z "$GCC_VERSION" ]; then
    export CC=gcc-$GCC_VERSION
    export CXX=g++-$GCC_VERSION
  fi

  # GTSAM_BUILD_WITH_MARCH_NATIVE=OFF: to avoid crashes in builder VMs
  # CMAKE_CXX_FLAGS="-w": Suppress warnings to avoid IO latency in CI logs
  cmake $SOURCE_DIR \
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
  [ -x "$(command -v ccache)" ] && ccache -s

  cd $SOURCE_DIR
}

# compile the code with the intent of populating the cache
function build ()
{
  export GTSAM_BUILD_EXAMPLES_ALWAYS=ON
  export GTSAM_BUILD_TESTS=OFF

  configure

  if [ "$(uname)" == "Linux" ]; then
    if (($(nproc) > 2)); then
      make -j4
    else
      make -j2
    fi
  elif [ "$(uname)" == "Darwin" ]; then
    make -j$(sysctl -n hw.physicalcpu)
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
      make -j$(nproc) check
    else
      make -j2 check
    fi
  elif [ "$(uname)" == "Darwin" ]; then
    make -j$(sysctl -n hw.physicalcpu) check
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