function geographiclibinterface(incdir, libdir)
% geographiclibinterface  Use mex to compile interface to GeographicLib
%
%   geographiclibinterface
%   geographiclibinterface(INSTALLDIR)
%   geographiclibinterface(INCDIR, LIBDIR)
%
% With one argument the library is looked for in INSTALLDIR/lib and the
% include files in INSTALLDIR/include.
%
% With no arguments, INSTALLDIR is taked to be '/usr/local', on Unix and
% Linux systems, and 'C:/Program Files/GeographicLib', on Windows systems
%
% With two arguments, the library is looked for in LIBDIR and the include
% files in INCDIR.
%
% This has been tested with
%
%   Octave 3.2.3 and g++ 4.4.4 under Linux
%   Matlab 2007a and Visual Studio 2005 under Windows
%   Matlab 2008a and Visual Studio 2005 under Windows
%   Matlab 2008a and Visual Studio 2008 under Windows (*)
%   Matlab 2010b and Visual Studio 2005 under Windows
%   Matlab 2010b and Visual Studio 2008 under Windows (*)
%   Matlab 2010b and Visual Studio 2010 under Windows
%
% (*) Note that geoidheight compiled with Visual Studio 2008 causes Matlab
%     to CRASH.  The crash happens on the second call.
%
% Run 'mex -setup' to configure the C++ compiler for Matlab to use.
%
% Copyright (c) Charles Karney (2010-2011) <charles@karney.com> and licensed
% under the MIT/X11 License.  For more information, see
% http://geographiclib.sf.net/html/other.html#matlab

  funs = {'geodesicdirect', 'geodesicinverse', 'geodesicline', ...
          'geoidheight', ...
          'utmupsforward', 'utmupsreverse', ...
          'mgrsforward', 'mgrsreverse', ...
          'geocentricforward', 'geocentricreverse', ...
          'localcartesianforward', 'localcartesianreverse', ...
          'polygonarea'};
  lib='Geographic';
  if (ispc && ...
      strcmp(mex.getCompilerConfigurations().Manufacturer, 'Microsoft') && ...
      strcmp(mex.getCompilerConfigurations().Version, '9.0'))
    warning(['geoidheight compiled with Visual Studio 2008 ',...
             'causes Matlab to CRASH']);
  end
  if (nargin < 2)
    if (nargin == 0)
      if ispc,
        installdir = 'C:/Program Files/GeographicLib';
      else
        installdir = '/usr/local';
      end
    else
      installdir = incdir;
    end
    incdir=[installdir '/include'];
    libdir=[installdir '/lib'];
  end
  testheader = [incdir '/GeographicLib/Constants.hpp'];
  if (~ exist(testheader, 'file'))
    error(['Cannot find ' testheader]);
  end
  fprintf('Compiling Matlab interface to GeographicLib\n');
  fprintf('Include directory: %s\nLibrary directory: %s\n', incdir, libdir);
  for i=1:size(funs,2),
    fprintf('Compiling %s...', funs{i});
    if ispc,
      mex( ['-I' incdir], ['-L' libdir], ['-l' lib], [funs{i} '.cpp'] );
    else
      mex( ['-I' incdir], ['-L' libdir], ['-l' lib], ['-Wl,-rpath=' libdir], ...
           [funs{i} '.cpp'] );
    end
    fprintf(' done.\n');
  end
end
