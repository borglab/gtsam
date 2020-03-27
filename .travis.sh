#!/bin/bash

# install TBB with _debug.so files
function install_tbb()
{
  TBBROOT=/tmp/tbb44_20151115oss

  if [ "$(uname -s)" == "Linux" ]; then
    wget https://github.com/oneapi-src/oneTBB/releases/download/4.4.2/tbb44_20151115oss_lin.tgz -O /tmp/tbb442.tgz
    tar -C /tmp -xf /tmp/tbb442.tgz

    TBB_TARGET_ARCH="intel64"
    library_directory="${TBB_TARGET_ARCH}/gcc4.1"

    # Set library paths
    export MIC_LD_LIBRARY_PATH="$TBBROOT/lib/mic:${MIC_LD_LIBRARY_PATH}"
    export MIC_LIBRARY_PATH="$TBBROOT/lib/mic:${MIC_LIBRARY_PATH}"
    export LD_LIBRARY_PATH="$TBBROOT/lib/$library_directory:${LD_LIBRARY_PATH}"
    export LIBRARY_PATH="$TBBROOT/lib/$library_directory:${LIBRARY_PATH}"

  elif [ "$(uname -s)" == "Darwin" ]; then
    wget https://github.com/oneapi-src/oneTBB/releases/download/4.4.2/tbb44_20151115oss_osx.tgz -O /tmp/tbb442.tgz
    tar -C /tmp -xf /tmp/tbb442.tgz

    # Set library paths
    export LIBRARY_PATH="${TBBROOT}/lib:$LIBRARY_PATH"
    export DYLD_LIBRARY_PATH="${TBBROOT}/lib:$DYLD_LIBRARY_PATH"
  fi

  CPATH="${TBBROOT}/include:$CPATH"; export CPATH
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
      -DCMAKE_VERBOSE_MAKEFILE=ON
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
  -tbb)
    install_tbb
    ;;
esac
