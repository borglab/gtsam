#!/bin/sh
echo "Installing GTSAM Python Wrapper"

PACKAGE_PATH=${GTSAM_CYTHON_INSTALL_PATH}${GTSAM_BUILD_TAG}

if [ ! -d "$PACKAGE_PATH" ]
then 
    echo "Directory $PACKAGE_PATH DOES NOT exist. Please run 'make install' first.";
    exit 1;
fi

# set cython directory permissions to user so we don't get permission denied
if [ "$(whoami)" != "root" ]
then
    sudo chown -R $(logname) ${GTSAM_CYTHON_INSTALL_PATH}
else
    chown -R $(logname) ${GTSAM_CYTHON_INSTALL_PATH}
fi

echo "Running setup.py in $PACKAGE_PATH"
${PYTHON_EXECUTABLE} $PACKAGE_PATH/setup.py install
