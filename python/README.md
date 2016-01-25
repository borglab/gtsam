Python Wrapper and Packaging
============================

This directory contains the basic setup script and directory structure for the gtsam python module.
During the build of gtsam, when GTSAM_BUILD_PYTHON is enabled, the following instructions will run.

* The handwritten module source files are compiled and linked with Boost Python, generating a shared 
  library which can then be imported by python
* A setup.py script is configured from setup.py.in
* The gtsam packages 'gtsam', 'gtsam_utils', 'gtsam_examples', and 'gtsam_tests' are installed into
  the site-packages folder within the (possibly non-default) installation prefix folder. If 
  installing to a non-standard prefix, make sure that _prefix_/lib/pythonX.Y/site-packages is 
  present in your $PYTHONPATH

The target version of Python to create the module can be set by defining GTSAM_PYTHON_VERSION to 'X.Y' (Example: 2.7 or 3.4), or 'Default' if you want to use the default python installed in your system. Note that if you specify a target version of python, you should also have the correspondening Boost 
Python version installed (Example: libboost_python-py27.so or libboost_python-py34.so on Linux). 
If you're using the default version, your default Boost Python library (Example: libboost_python.so on Linux) should correspond to the default python version in your system.

