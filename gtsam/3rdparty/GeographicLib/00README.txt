A library for geographic projections.

Written by Charles Karney <charles@karney.com> and licensed under
the MIT/X11 License.  For more information, see

    https://geographiclib.sourceforge.io/

Files

    00README.txt  -- this file
    AUTHORS -- the authors of the library
    LICENSE.txt -- the MIT/X11 License
    INSTALL -- brief installation instructions
    NEWS -- a history of changes

    include/GeographicLib/*.hpp
      headers for the library
    src/*.cpp
      implementation for the library

    examples/
      examples for each class

    tools/
      command-line utilities

    Makefile.mk -- Unix/Linux makefiles
    configure -- autoconf configuration script
    CMakeLists.txt -- cmake configuration files
    cmake/
      support files for building with CMake

    windows/
      project files for building under Windows (but CMake is preferred)

    maxima/
      Maxima code for generating series expansions, etc.

    matlab/
      geographiclib/
        *.m, private/*.m -- Matlab implementation of some classes
      geographiclib-legacy/
        *.{m,cpp} -- legacy Matlab routines

    doc/
      files for generating documentation with Doxygen

    man/
      man pages for the utilities

    python/GeographicLib/*.py -- Python implementation of geodesic routines

    java/.../*.java -- Java implementation of geodesic routines

    js/
      src/*.js -- JavaScript implementation of geodesic routines
      samples/*.html -- demonstrations of the JavaScript interface

    legacy/
      C/ -- C implementation of geodesic routines
      Fortran/ -- Fortran implementation of geodesic routines

    dotnet/
      NETGeographicLib/*.{cpp,h} -- .NET wrapper for GeographicLib
      examples/
        CS/*.cs -- simple C# examples for each class
        ManagedCPP/*.cpp -- Managed C++ examples for each class
        VB/*.vb -- simple Visual Basic examples for each class
      Projection/ -- a more complex C# application
