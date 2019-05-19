The geodesic routines in GeographicLib have been implemented as a native
Python library.  See

  https://geographiclib.sourceforge.io/html/python/

It is also possible to call the C++ version of GeographicLib directly
from Python and this directory contains a small example,
PyGeographicLib.cpp, which uses boost-python and the Geoid class to
convert heights above the geoid to heights above the ellipsoid.  More
information on calling boost-python, see

  http://www.boost.org/doc/libs/release/libs/python

To build and install this interface, do

  mkdir BUILD
  cd BUILD
  cmake -D CMAKE_INSTALL_PREFIX=~/.local ..
  make
  make install

This assumes that you have installed GeographicLib somewhere that cmake
can find it.  If you want just to use the version of GeographicLib that
you have built in the top-level BUILD directory, include, e.g.,

  -D GeographicLib_DIR=../../BUILD

in the invocation of cmake (the directory is relative to the source
directory, wrapper/python).

"make install" installs PyGeographicLib in

  ~/.local/lib/python2.7/site-packages

which is in the default search path for python 2.7.  To convert 20m
above the geoid at 42N 75W to a height above the ellipsoid, do

  $ python
  >>> from PyGeographicLib import Geoid
  >>> geoid = Geoid("egm2008-1")
  >>> geoid.EllipsoidHeight(42, -75, 20)
  -10.671887499999997
  >>> help(Geoid.EllipsoidHeight)

Notes:

* The geoid data (egm2008-1) should be installed somewhere that
  GeographicLib knows about.

* This prescription applies to Linux machines.  Similar steps can be
  used on Windows and MacOSX machines.

* You will need the packages boost-python, boost-devel, python, and
  python-devel installed.

* CMakeLists.txt specifies the version of python to look for (version
  2.7).  This must match that used in boost-python.  To check do, e.g.,

    ldd /usr/lib64/libboost_python.so

* CmakeLists.txt looks for a shared-library version of GeographicLib.
  This is the default with cmake build on non-Windows platforms.  On
  Windows, use the cmake variable GEOGRAPHICLIB_LIB_TYPE to specify
  building a shared library.

Acknowledgment:

Thanks to Jonathan Takahashi <jtakahashi@gmail.com> for the sample code
in PyGeographicLib.cpp and the commands needed to compile and link this.
