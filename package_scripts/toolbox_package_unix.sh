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
-DGTSAM_INSTALL_MATLAB_TOOLBOX:BOOL=ON \
-DCMAKE_INSTALL_PREFIX="$PWD/stage" \
-DBoost_NO_SYSTEM_PATHS:BOOL=ON \
-DBoost_USE_STATIC_LIBS:BOOL=ON \
-DBOOST_ROOT="$1" \
-DGTSAM_BUILD_TESTS:BOOL=OFF \
-DGTSAM_BUILD_TIMING:BOOL=OFF \
-DGTSAM_BUILD_EXAMPLES_ALWAYS:BOOL=OFF \
-DGTSAM_WITH_TBB:BOOL=OFF \
-DGTSAM_BUILD_METIS:BOOL=OFF \
-DGTSAM_INSTALL_GEOGRAPHICLIB:BOOL=OFF \
-DGTSAM_BUILD_UNSTABLE:BOOL=OFF \
-DGTSAM_MEX_BUILD_STATIC_MODULE:BOOL=ON ..

if [ $? -ne 0 ]; then
	echo "CMake failed"
	exit 1
fi

# Compile
make -j8 install

if [ $? -ne 0 ]; then
    echo "Compile failed"
    exit 1
fi

# Create package
tar czf gtsam-toolbox-3.2.0-$platform.tgz -C stage/gtsam_toolbox toolbox
