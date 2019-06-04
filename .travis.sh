#!/bin/bash

set -e   # Make sure any error makes the script to return an error code
set -x   # echo

SOURCE_DIR=`pwd`
BUILD_DIR=build

#CMAKE_C_FLAGS="-Wall -Wextra -Wabi -O2"
#CMAKE_CXX_FLAGS="-Wall -Wextra -Wabi -O2"

function build_and_test ()
{
  #env
  git clean -fd || true
  rm -fr $BUILD_DIR || true
  mkdir $BUILD_DIR && cd $BUILD_DIR

  if [ ! -z "$GCC_VERSION" ]; then
    sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-$GCC_VERSION 60 \
                         --slave /usr/bin/g++ g++ /usr/bin/g++-$GCC_VERSION
    sudo update-alternatives --set gcc /usr/bin/gcc-$GCC_VERSION
  fi

  cmake $SOURCE_DIR \
      -DGTSAM_BUILD_TESTS=$GTSAM_BUILD_TESTS \
      -DGTSAM_BUILD_UNSTABLE=$GTSAM_BUILD_UNSTABLE \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=$GTSAM_BUILD_EXAMPLES_ALWAYS \
      -DGTSAM_ALLOW_DEPRECATED_SINCE_V4=$GTSAM_ALLOW_DEPRECATED_SINCE_V4

  # Actual build:
  make -j2

  # Run tests:
  if [ "$GTSAM_BUILD_TESTS" == "ON" ]; then
    make check
  fi

  # Print ccache stats
  ccache -s

  cd $SOURCE_DIR
}

build_and_test
