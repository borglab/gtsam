#!/bin/bash

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

    if [ ! -z "$GCC_VERSION" ]; then
      export CC=gcc-$GCC_VERSION
      export CXX=g++-$GCC_VERSION
    fi

	# GTSAM_BUILD_WITH_MARCH_NATIVE=OFF: to avoid crashes in builder VMs

	cmake $SOURCE_DIR \
        -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE:-Debug} \
        -DGTSAM_BUILD_TESTS=${GTSAM_BUILD_TESTS:-Off} \
        -DGTSAM_BUILD_UNSTABLE=${GTSAM_BUILD_UNSTABLE:-On} \
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=${GTSAM_BUILD_EXAMPLES_ALWAYS:-On} \
        -DGTSAM_ALLOW_DEPRECATED_SINCE_V4=${GTSAM_ALLOW_DEPRECATED_SINCE_V4:-OFF} \
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
        -DCMAKE_VERBOSE_MAKEFILE=On
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
  export GTSAM_BUILD_EXAMPLES_ALWAYS=On
  export GTSAM_BUILD_TESTS=Off

  configure

  make -j2

  finish
}

# run the tests
function test ()
{
  export GTSAM_BUILD_EXAMPLES_ALWAYS=Off
  export GTSAM_BUILD_TESTS=On

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
