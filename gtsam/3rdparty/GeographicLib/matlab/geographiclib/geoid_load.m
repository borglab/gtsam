function geoid = geoid_load(name, dir)
%GEOID_LOAD  Load a geoid model
%
%   geoid = GEOID_LOAD
%   geoid = GEOID_LOAD(geoidname)
%   geoid = GEOID_LOAD(geoidname, geoiddir)
%
%   Loads geoid data into the workspace.  geoid_height uses this function
%   to load the geoid data it needs.  The possible geoids are
%
%       egm84-30  egm84-15
%       egm96-15  egm96-5
%       egm2008-5 egm2008-2_5 egm2008-1
%
%   The first part of the name is the geoid model.  The second part gives
%   the resolution of the gridded data (in arc-seconds).  Thus egm2008-2_5
%   is the egm2008 geoid model at a resolution of 2.5".
%
%   If geoidname is not specified (or is empty), the environment variable
%   GEOGRAPHICLIB_GEOID_NAME is used; if this is not defined then egm96-5
%   is used.
%
%   GEOID_LOAD determines the directory with the geoid data as follows
%   (here an empty string is the same as undefined):
%
%      * if geoiddir is specified, then look there; otherwise ...
%      * if the environment variable GEOGRAPHICLIB_GEOID_PATH is defined,
%        look there; otherwise ...
%      * if the environment variable GEOGRAPHICLIB_DATA is defined, look in
%        [GEOGRAPHICLIB_DATA '/geoids']; otherwise ...
%      * look in /usr/local/share/GeographicLib/geoids (for non-Windows
%        systems) or C:/ProgramData/GeographicLib/geoids (for Windows
%        systems).
%
%   If your geoid models are installed in /usr/share/GeographicLib/geoids,
%   for example, you can avoid the need to supply the geoiddir argument
%   with
%
%       setenv GEOGRAPHICLIB_DATA /usr/share/GeographicLib
%
%   The geoid data is loaded from the image file obtained by concatenating
%   the components to give geoiddir/geoidname.pgm.  These files store a
%   grid of geoid heights above the ellipsoid encoded as 16-bit integers.
%
%   The returned geoid can be passed to geoid_height to determine the
%   height of the geoid above the ellipsoid.
%
%   Information on downloading and installing the data for the supported
%   geoid models is available at
%
%     https://geographiclib.sourceforge.io/html/geoid.html#geoidinst
%
%   See also GEOID_HEIGHT.

% Copyright (c) Charles Karney (2015) <charles@karney.com>.

  if nargin < 1
    file = geoid_file;
  elseif nargin < 2
    file = geoid_file(name);
  else
    file = geoid_file(name, dir);
  end
  geoid = geoid_load_file(file);
end
