function [lat, lon, azi, rk] = cassini_inv(lat0, lon0, x, y, ellipsoid)
%CASSINI_INV  Inverse Cassini-Soldner projection
%
%   [lat, lon] = CASSINI_INV(lat0, lon0, x, y)
%   [lat, lon, azi, rk] = CASSINI_INV(lat0, lon0, x, y, ellipsoid)
%
%   performs the inverse Cassini-Soldner projection of points (x,y) to
%   (lat,lon) using (lat0,lon0) as the center of projection.  These input
%   arguments can be scalars or arrays of equal size.  The ellipsoid vector
%   is of the form [a, e], where a is the equatorial radius in meters, e is
%   the eccentricity.  If ellipsoid is omitted, the WGS84 ellipsoid (more
%   precisely, the value returned by defaultellipsoid) is used.  projdoc
%   defines the projection and gives the restrictions on the allowed ranges
%   of the arguments.  The forward projection is given by cassini_fwd.
%
%   azi and rk give metric properties of the projection at (lat,lon); azi
%   is the azimuth of the easting (x) direction and rk is the reciprocal of
%   the northing (y) scale.  The scale in the easting direction is 1.
%
%   lat0, lon0, lat, lon, azi are in degrees.  The projected coordinates x,
%   y are in meters (more precisely the units used for the equatorial
%   radius).  rk is dimensionless.
%
%   See also PROJDOC, CASSINI_FWD, GEODRECKON, DEFAULTELLIPSOID, FLAT2ECC.

% Copyright (c) Charles Karney (2012-2015) <charles@karney.com>.

  narginchk(4, 5)
  if nargin < 5, ellipsoid = defaultellipsoid; end
  try
    [~] = lat0 + lon0 + x + y;
  catch
    error('lat0, lon0, x, y have incompatible sizes')
  end

  [lat1, lon1, azi0] = geodreckon(lat0, lon0, y, 0, ellipsoid);
  [lat, lon, azi, ~, ~, rk] = ...
      geodreckon(lat1, lon1, x, azi0 + 90, ellipsoid);
end
