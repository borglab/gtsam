# Calling GeographicLib C++ library from Octave/MATLAB

The Octave/MATLAB package
[geographiclib](https://github.com/geographiclib/geographiclib-octave#readme)
provides a native Octave/MATLAB implementation of a subset of
GeographicLib.  This is also available via the MATLAB Central Package
[geographiclib](https://www.mathworks.com/matlabcentral/fileexchange/50605)

It is also possible to call the C++ GeographicLib library directly
from Octave and MATLAB.  This gives you access to the full range of
GeographicLib's capabilities.

In order to make use of this facility, it is necessary to write some
interface code.  The files in this directory provide a sample of such
interface code.  This example solves the inverse geodesic problem for
ellipsoids with arbitrary flattening.  (The code `geoddistance.m` does
this as native Matlab code; but it is limited to ellipsoids with a
smaller flattening.)

For full details on how to write the interface code, see

  https://www.mathworks.com/help/matlab/write-cc-mex-files.html

To compile the interface code, start Octave or MATLAB and run, e.g.,
```Octave
mex -setup C++
help geographiclibinterface
geographiclibinterface
help geodesicinverse
geodesicinverse([40.6, -73.8, 51.6, -0.5])
ans =

   5.1199e+01   1.0782e+02   5.5518e+06
```

The first command allows you to select the compiler to use (which should
be the same as that used to compile GeographicLib).

These routines just offer a simple interface to the corresponding C++
class. Use the help function to get documentation,
```Octave
help geodesicinverse
```

Unfortunately, the help function does not work for compiled functions in
Octave; in this case, just list the .m file, e.g.,
```Octave
type geodesicinverse.m
```
