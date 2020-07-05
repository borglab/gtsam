function [lat, lon, azi, rk] = eqdazim_inv(lat0, lon0, x, y, ellipsoid)
%EQDAZIM_INV  Inverse azimuthal equidistant projection
%
%   [lat, lon] = EQDAZIM_INV(lat0, lon0, x, y)
%   [lat, lon, azi, rk] = EQDAZIM_INV(lat0, lon0, x, y, ellipsoid)
%
%   performs the inverse azimuthal equidistant projection of points (x,y)
%   to (lat,lon) using (lat0,lon0) as the center of projection.  These
%   input arguments can be scalars or arrays of equal size.  The ellipsoid
%   vector is of the form [a, e], where a is the equatorial radius in
%   meters, e is the eccentricity.  If ellipsoid is omitted, the WGS84
%   ellipsoid (more precisely, the value returned by defaultellipsoid) is
%   used.  projdoc defines the projection and gives the restrictions on
%   the allowed ranges of the arguments.  The forward projection is given
%   by eqdazim_fwd.
%
%   azi and rk give metric properties of the projection at (lat,lon); azi
%   is the azimuth of the geodesic from the center of projection and rk is
%   the reciprocal of the azimuthal scale.  The scale in the radial
%   direction is 1.
%
%   lat0, lon0, lat, lon, azi are in degrees.  The projected coordinates x,
%   y are in meters (more precisely the units used for the equatorial
%   radius).  rk is dimensionless.
%
%   Section 14 of
%
%    C. F. F. Karney, Geodesics on an ellipsoid of revolution (2011),
%    https://arxiv.org/abs/1102.1215
%    Errata: https://geographiclib.sourceforge.io/geod-addenda.html#geod-errata
%
%   describes how to use this projection in the determination of maritime
%   boundaries (finding the median line).
%
%   See also PROJDOC, EQDAZIM_FWD, GEODRECKON, DEFAULTELLIPSOID, FLAT2ECC.

% Copyright (c) Charles Karney (2012-2015) <charles@karney.com>.

  narginchk(4, 5)
  if nargin < 5, ellipsoid = defaultellipsoid; end
  try
    [~] = lat0 + lon0 + x + y;
  catch
    error('lat0, lon0, x, y have incompatible sizes')
  end

  azi0 = atan2dx(x, y);
  s = hypot(x, y);
  [lat, lon, azi, ~, m, ~, ~, sig] = ...
      geodreckon(lat0, lon0, s, azi0, ellipsoid);
  rk = m ./ s;
  rk(sig <= 0.01 * sqrt(realmin)) = 1;
end
