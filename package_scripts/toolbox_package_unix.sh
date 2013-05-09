#!/bin/sh

# Script to build a tarball with the matlab toolbox

# Detect platform
os=`uname -s`
arch=`uname -m`
if [ "$os" = "Linux" -a "$arch" = "x86_64" ]; then
	platform=lin64
elif [ "$os" = "Linux" -a "$arch" = "i686" ]; then
	platform=lin32
elif [ "$os" = "Darwin" -a "$arch" = "x86_64" ]; then
	platform=mac64
else
	echo "Unrecognized platform"
	exit 1
fi

echo "Platform is ${platform}"

# Check for empty directory
if [ ! -z "`ls`" ]; then
	echo "Please run this script from an empty build directory"
	exit 1
fi

# Check for boost
if [ -z "$1" ]; then
	echo "Usage: $0 BOOSTTREE"
	echo "BOOSTTREE should be a boost source tree compiled with toolbox_build_boost."
	exit 1
fi

# Run cmake
cmake -DCMAKE_BUILD_TYPE=Release \
-DGTSAM_INSTALL_MATLAB_TOOLBOX:bool=true \
-DCMAKE_INSTALL_PREFIX="$PWD/stage" \
-DBoost_NO_SYSTEM_PATHS:bool=true \
-DBoost_USE_STATIC_LIBS:bool=true \
-DBOOST_ROOT="$1" \
-DGTSAM_BUILD_SHARED_LIBRARY:bool=false \
-DGTSAM_BUILD_STATIC_LIBRARY:bool=true \
-DGTSAM_BUILD_TESTS:bool=false \
-DGTSAM_BUILD_EXAMPLES:bool=false \
-DGTSAM_BUILD_UNSTABLE:bool=false \
-DGTSAM_DISABLE_EXAMPLES_ON_INSTALL:bool=true \
-DGTSAM_DISABLE_TESTS_ON_INSTALL:bool=true \
-DGTSAM_BUILD_CONVENIENCE_LIBRARIES:bool=false \
-DGTSAM_MEX_BUILD_STATIC_MODULE:bool=true ..

if [ ! $? ]; then
	echo "CMake failed"
	exit 1
fi

# Compile
make -j8 install

# Create package
tar czf gtsam-toolbox-2.3.0-$platform.tgz -C stage/borg toolbox
