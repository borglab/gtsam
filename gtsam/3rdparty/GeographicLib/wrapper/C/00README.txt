The geodesic routines in GeographicLib have been implemented as a native
C library.  See

  https://geographiclib.sourceforge.io/html/C/

It is also possible to call the C++ version of GeographicLib directly
from C and this directory contains a small example, which convert
heights above the geoid to heights above the ellipsoid.  More
information on calling C++ from C, see

  https://isocpp.org/wiki/faq/mixing-c-and-cpp

To build and install this interface, do

  mkdir BUILD
  cd BUILD
  cmake ..
  make

This assumes that you have installed GeographicLib somewhere that cmake
can find it.  If you want just to use the version of GeographicLib that
you have built in the top-level BUILD directory, include, e.g.,

  -D GeographicLib_DIR=../../BUILD

in the invocation of cmake (the directory is relative to the source
directory, wrapper/C).  To convert 20m above the geoid at 42N 75W to a
height above the ellipsoid, use

$ echo 42 -75 20 | ./geoidtest
-10.672

Notes:

* The geoid data (egm2008-1) should be installed somewhere that
  GeographicLib knows about.

* This prescription applies to Linux machines.  Similar steps can be
  used on Windows and MacOSX machines.

* It is essential that the application be linked with the C++ compiler,
  so that the C++ runtime library is included.

* In this example, the main program is compiled with the C compiler.  In
  more complicated situations, it may be necessary to use the C++
  compiler.  This is necessary to get static initializations of C++
  classes performed.  (However, GeographicLib doesn't need any static
  initialization.)
