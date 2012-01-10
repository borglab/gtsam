#!/bin/sh

# Add to PATH the default install path for doxygen on Mac OS X

PATH=$PATH:/Applications/Doxygen.app/Contents/Resources


# Run doxygen from the gtsam directory even if this script is run from another
# directory, so that the output 'doc' goes in the gtsam directory.

GTSAM_DIR=$(dirname "$0")
cd $GTSAM_DIR
doxygen