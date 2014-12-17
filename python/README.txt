This directory contains the basic setup script and directory structure for the gtsam python module.
During the build of gtsam, when GTSAM_BUILD_PYTHON is enabled, the following instructions will run.
* Wrap parses gtsam.h and constructs a cpp file called ${moduleName}_python.cpp
* This file is then compiled and linked with BoostPython, generating a shared library which can then be imported by python
* The shared library is then copied to python/gtsam
* The user can use the setup.py script to build and install a python package, allowing easy importing into a python project. Examples:
  * python setup.py sdist   ---- Builds a tarball of the python package which can then be distributed
  * python setup.py install ---- Installs the package into the python dist-packages folder. Can then be imported from any python file.
* To run the unit tests, you must first install the package on your path (TODO: Make this easier)


TODO: There are many issues with this build system, but these are the basics.
