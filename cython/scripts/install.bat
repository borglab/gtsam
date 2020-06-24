:: This script runs the installation flow for python wrapped GTSAM.
:: It does so by running `python setup.py install` to install the wrapped package.

echo "Installing GTSAM Python Wrapper"

:: Set the package path
PACKAGE_PATH=${GTSAM_CYTHON_INSTALL_PATH}${GTSAM_BUILD_TAG}

:: Check if package directory exists. If not, print warning and exit.
if [ ! -d "$PACKAGE_PATH" ]
then
    echo "Directory $PACKAGE_PATH DOES NOT exist. Please run 'make install' first.";
    exit 1;
fi

:: Run setup.py install with full paths.
echo "Running setup.py in $PACKAGE_PATH"
${PYTHON_EXECUTABLE} $PACKAGE_PATH/setup.py install
