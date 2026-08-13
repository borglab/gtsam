#!/bin/bash

set -e
set -x

PYTHON_VERSION="$1"
PROJECT_DIR="$2"
BOOST_BUILD_JOBS=2
GTSAM_BUILD_JOBS=2

# The generated Python wrapper translation units require substantial memory.
# Compile them serially in Linux wheel containers to avoid the OOM killer.
if [ "$(uname)" == "Linux" ]; then
    GTSAM_BUILD_JOBS=1
fi

echo "Build parallelism: Boost=${BOOST_BUILD_JOBS}, GTSAM=${GTSAM_BUILD_JOBS}"

export PYTHON="python${PYTHON_VERSION}"

if [ "$(uname)" == "Linux" ]; then
    yum install -y wget
elif [ "$(uname)" == "Darwin" ]; then
    brew install wget

    if [[ -z "${MACOSX_DEPLOYMENT_TARGET}" ]]; then
        export MACOSX_DEPLOYMENT_TARGET="$(sw_vers -productVersion | cut -d '.' -f 1-2)"
    fi
fi

wget https://archives.boost.io/release/1.87.0/source/boost_1_87_0.tar.gz --quiet
tar -xzf boost_1_87_0.tar.gz
cd boost_1_87_0

BOOST_PREFIX="$HOME/opt/boost"
./bootstrap.sh --prefix=${BOOST_PREFIX}

if [ "$(uname)" == "Linux" ]; then
    ./b2 -j${BOOST_BUILD_JOBS} install --prefix=${BOOST_PREFIX} -d0 --with-graph \
        --with-move --with-optional --with-program_options --with-random \
        --with-serialization --with-smart_ptr --with-timer --with-chrono \
        --with-filesystem
elif [ "$(uname)" == "Darwin" ]; then
    BOOST_ARCH="${GTSAM_BOOST_ARCH:-arm}"
    ./b2 -j${BOOST_BUILD_JOBS} install --prefix=${BOOST_PREFIX} -d0 --with-graph \
        --with-move --with-optional --with-program_options --with-random \
        --with-serialization --with-smart_ptr --with-timer --with-chrono \
        --with-filesystem \
        architecture=${BOOST_ARCH} \
        cxxflags="-mmacosx-version-min=${MACOSX_DEPLOYMENT_TARGET}" \
        linkflags="-mmacosx-version-min=${MACOSX_DEPLOYMENT_TARGET}"
fi
cd ..

export BOOST_ROOT="${BOOST_PREFIX}"
export BOOST_INCLUDEDIR="${BOOST_PREFIX}/include"
export BOOST_LIBRARYDIR="${BOOST_PREFIX}/lib"
export LD_LIBRARY_PATH="${BOOST_LIBRARYDIR}:$LD_LIBRARY_PATH"
export REPAIR_LIBRARY_PATH="${BOOST_LIBRARYDIR}:$DYLD_LIBRARY_PATH"

if [ "$(uname)" == "Darwin" ]; then
    for dylib in ${BOOST_LIBRARYDIR}/*.dylib; do
        install_name_tool -add_rpath "@loader_path" "$dylib"
    done
fi

$(which $PYTHON) -m pip install -r $PROJECT_DIR/python/dev_requirements.txt

rm -rf $PROJECT_DIR/build
rm -rf CMakeCache.txt CMakeFiles

cmake $PROJECT_DIR \
    -B build \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} \
    -DCMAKE_OSX_ARCHITECTURES="${GTSAM_OSX_ARCHITECTURES:-arm64;x86_64}" \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_BUILD_UNSTABLE=${GTSAM_BUILD_UNSTABLE:-ON} \
    -DGTSAM_USE_QUATERNIONS=OFF \
    -DGTSAM_WITH_TBB=${GTSAM_WITH_TBB:-OFF} \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_BUILD_PYTHON=ON \
    -DGTSAM_UNSTABLE_BUILD_PYTHON=${GTSAM_BUILD_UNSTABLE:-ON} \
    -DGTSAM_PYTHON_VERSION=$PYTHON_VERSION \
    -DPYTHON_EXECUTABLE:FILEPATH=$(which $PYTHON) \
    -DGTSAM_ALLOW_DEPRECATED_SINCE_V42=OFF \
    -DCMAKE_INSTALL_PREFIX=$PROJECT_DIR/gtsam_install

cd $PROJECT_DIR/build
make -j${GTSAM_BUILD_JOBS} install
