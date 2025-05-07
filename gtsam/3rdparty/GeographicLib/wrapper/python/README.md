# Calling the GeographicLib C++ library from Python

The geodesic routines in GeographicLib have been implemented as a
[native Python library](http://pypi.python.org/pypi/geographiclib).
For documentation see

  https://geographiclib.sourceforge.io/Python/doc/

## ctypes

This is the official way to use C++ libraries from Python.  See

  https://docs.python.org/3/extending/extending.html#writing-extensions-in-c

Borys Kaboakov https://github.com/banderlog provided a sample invocation
in the PyGetIntersect directory.

## pybind11

See

  https://github.com/pybind/pybind11

This seems to be a good alternative to boost-python.  Still need to
provide spccific instructions.

## boost-python

It is also possible to call the C++ version of GeographicLib directly
from Python and this directory contains a small example,
`PyGeographicLib.cpp`, which uses boost-python and the `Geoid` class to
convert heights above the geoid to heights above the ellipsoid.  More
information on calling boost-python, see

  https://www.boost.org/doc/libs/release/libs/python

To build and install this interface, do
```bash
mkdir BUILD
cd BUILD
cmake -D CMAKE_INSTALL_PREFIX=~/.local ..
make
make install
```
This assumes that you have installed GeographicLib somewhere that cmake
can find it.  If you want just to use the version of GeographicLib that
you have built in the top-level BUILD directory, include, e.g.,
```bash
-D GeographicLib_DIR=../../BUILD
```
in the invocation of cmake (the directory is relative to the source
directory, wrapper/python).

`make install` installs PyGeographicLib in
```
~/.local/lib/python2.7/site-packages
```
which is in the default search path for python 2.7.  To convert 20m
above the geoid at 42N 75W to a height above the ellipsoid, do
```python
$ python
>>> from PyGeographicLib import Geoid
>>> geoid = Geoid("egm2008-1")
>>> geoid.EllipsoidHeight(42, -75, 20)
-10.671887499999997
>>> help(Geoid.EllipsoidHeight)
```

Notes:

* The geoid data (`egm2008-1`) should be installed somewhere that
  GeographicLib knows about.

* This prescription applies to Linux machines.  Similar steps can be
  used on Windows and MacOSX machines.

* You will need the packages boost-python, boost-devel, python, and
  python-devel installed.

* `CMakeLists.txt` specifies the version of python to look for (version
  2.7).  This must match that used in boost-python.  To check do, e.g.,
  ```bash
  ldd /usr/lib64/libboost_python.so
  ```

* `CMakeLists.txt` looks for a shared-library version of GeographicLib.
  This is the default with cmake build on non-Windows platforms.  On
  Windows, use the cmake option `-D BUILD_SHARED_LIBS=ON` to specify
  building a shared library.

Acknowledgment:

Thanks to Jonathan Takahashi <jtakahashi@gmail.com> for the sample code
in PyGeographicLib.cpp and the commands needed to compile and link this.

## Cython

An alternative to boost-python is provided by [Cython](https::/cython.org).
Sergey Serebryakov offers the github project

  https://github.com/megaserg/geographiclib-cython-bindings

which provides a fast python interface to the geodesic routines.
