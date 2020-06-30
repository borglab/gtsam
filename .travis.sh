#!/bin/bash

# install TBB with _debug.so files
function install_tbb()
{
  TBB_BASEURL=https://github.com/oneapi-src/oneTBB/releases/download
  TBB_VERSION=4.4.2
  TBB_DIR=tbb44_20151115oss
  TBB_SAVEPATH="/tmp/tbb.tgz"

  if [ "$TRAVIS_OS_NAME" == "linux" ]; then
    OS_SHORT="lin"
    TBB_LIB_DIR="intel64/gcc4.4"
    SUDO="sudo"

  elif [ "$TRAVIS_OS_NAME" == "osx" ]; then
    OS_SHORT="lin"
    TBB_LIB_DIR=""
    SUDO=""

  fi

  wget "${TBB_BASEURL}/${TBB_VERSION}/${TBB_DIR}_${OS_SHORT}.tgz" -O $TBB_SAVEPATH
  tar -C /tmp -xf $TBB_SAVEPATH

  TBBROOT=/tmp/$TBB_DIR
  # Copy the needed files to the correct places.
  # This works correctly for travis builds, instead of setting path variables.
  # This is what Homebrew does to install TBB on Macs
  $SUDO cp -R $TBBROOT/lib/$TBB_LIB_DIR/* /usr/local/lib/
  $SUDO cp -R $TBBROOT/include/ /usr/local/include/

}

# common tasks before either build or test
function configure()
{
  set -e   # Make sure any error makes the script to return an error code
  set -x   # echo

  SOURCE_DIR=`pwd`
  BUILD_DIR=build

  #env
  git clean -fd || true
  rm -fr $BUILD_DIR || true
  mkdir $BUILD_DIR && cd $BUILD_DIR

  install_tbb

  if [ ! -z "$GCC_VERSION" ]; then
    export CC=gcc-$GCC_VERSION
    export CXX=g++-$GCC_VERSION
  fi

  # GTSAM_BUILD_WITH_MARCH_NATIVE=OFF: to avoid crashes in builder VMs
  cmake $SOURCE_DIR \
      -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE:-Debug} \
      -DGTSAM_BUILD_TESTS=${GTSAM_BUILD_TESTS:-OFF} \
      -DGTSAM_BUILD_UNSTABLE=${GTSAM_BUILD_UNSTABLE:-ON} \
      -DGTSAM_WITH_TBB=${GTSAM_WITH_TBB:-OFF} \
      -DGTSAM_USE_QUATERNIONS=${GTSAM_USE_QUATERNIONS:-OFF} \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=${GTSAM_BUILD_EXAMPLES_ALWAYS:-ON} \
      -DGTSAM_ALLOW_DEPRECATED_SINCE_V4=${GTSAM_ALLOW_DEPRECATED_SINCE_V4:-OFF} \
      -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
      -DCMAKE_VERBOSE_MAKEFILE=OFF
}


# common tasks after either build or test
function finish ()
{
  # Print ccache stats
  ccache -s

  cd $SOURCE_DIR
}

# compile the code with the intent of populating the cache
function build ()
{
  export GTSAM_BUILD_EXAMPLES_ALWAYS=ON
  export GTSAM_BUILD_TESTS=OFF

  configure

  make -j2

  finish
}

# run the tests
function test ()
{
  export GTSAM_BUILD_EXAMPLES_ALWAYS=OFF
  export GTSAM_BUILD_TESTS=ON

  configure

  # Actual build:
  make -j2 check

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
