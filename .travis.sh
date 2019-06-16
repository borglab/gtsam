#!/bin/bash

# common tasks before either build or test
function prepare ()
{
  set -e   # Make sure any error makes the script to return an error code
  set -x   # echo

  SOURCE_DIR=`pwd`
  BUILD_DIR=build

  #env
  git clean -fd || true
  rm -fr $BUILD_DIR || true
  mkdir $BUILD_DIR && cd $BUILD_DIR

  if [ -z "$CMAKE_BUILD_TYPE" ]; then
    CMAKE_BUILD_TYPE=Debug
  fi

  if [ -z "$GTSAM_ALLOW_DEPRECATED_SINCE_V4" ]; then
    GTSAM_ALLOW_DEPRECATED_SINCE_V4=OFF
  fi

  if [ ! -z "$GCC_VERSION" ]; then
    sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-$GCC_VERSION 60 \
                         --slave /usr/bin/g++ g++ /usr/bin/g++-$GCC_VERSION
    sudo update-alternatives --set gcc /usr/bin/gcc-$GCC_VERSION
  fi
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
  prepare

  cmake $SOURCE_DIR \
      -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE \
      -DGTSAM_BUILD_TESTS=OFF \
      -DGTSAM_BUILD_UNSTABLE=$GTSAM_BUILD_UNSTABLE \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=ON \
      -DGTSAM_ALLOW_DEPRECATED_SINCE_V4=$GTSAM_ALLOW_DEPRECATED_SINCE_V4

  # Actual build:
  VERBOSE=1 make -j2

  finish
}

# run the tests
function test ()
{
  prepare

  cmake $SOURCE_DIR \
      -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE \
      -DGTSAM_BUILD_TESTS=ON \
      -DGTSAM_BUILD_UNSTABLE=$GTSAM_BUILD_UNSTABLE \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
      -DGTSAM_ALLOW_DEPRECATED_SINCE_V4=OFF

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
