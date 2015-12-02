Python Wrapper and Packaging
============================

This directory contains the basic setup script and directory structure for the gtsam python module.
During the build of gtsam, when GTSAM_BUILD_PYTHON is enabled, the following instructions will run.
* The python files that compose the module are copied from python/gtsam to $BUILD_DIR/python/gtsam
* The handwritten module source files are then compiled and linked with Boost Python, generating a shared library which can then be imported by python
* The shared library is then copied to $BUILD_DIR/python/gtsam and renamed with a "_" prefix 
* The user can use the setup.py script inside $BUILD_DIR/python to build and install a python package, allowing easy importing into a python project. Examples (when run from $BUILD_DIR):
  * python setup.py sdist   ---- Builds a tarball of the python package which can then be distributed
  * python setup.py install ---- Installs the package into the python dist-packages folder. Can then be imported from any python file.
  * python setup.py install --prefix="your/local/install/path"---- Installs the package into a local instalation folder. Can then be imported from any python file if _prefix_/lib/pythonX.Y/site-packages is present in your $PYTHONPATH

* To run the unit tests, you must first install the package on your path (TODO: Make this easier)

The target version of Python to create the module can be set by defining GTSAM_PYTHON_VERSION to 'X.Y' (Example: 2.7 or 3.4), or 'Default' if you want to use the default python installed in your system. Note that if you specify a target version of python, you should also have the correspondent Boost Python version installed (Example: libboost_python-py27.so or libboost_python-py34.so on Linux). If you're using the default version, your default Boost Python library (Example: libboost_python.so on Linux) should correspond to the default python version in your system.

TODO: There are many issues with this build system, but these are the basics.
