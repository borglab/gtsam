function [lat, lon, azi, rk] = eqdazim_inv(lat0, lon0, x, y, ellipsoid)
%EQDAZIM_INV  Inverse azimuthal equidistant projection
%
%   [LAT, LON] = EQDAZIM_INV(LAT0, LON0, X, Y)
%   [LAT, LON, AZI, RK] = EQDAZIM_INV(LAT0, LON0, X, Y, ELLIPSOID)
%
%   performs the inverse azimuthal equidistant projection of points (X,Y)
%   to (LAT,LON) using (LAT0,LON0) as the center of projection.  These
%   input arguments can be scalars or arrays of equal size.  The ELLIPSOID
%   vector is of the form [a, e], where a is the equatorial radius in
%   meters, e is the eccentricity.  If ellipsoid is omitted, the WGS84
%   ellipsoid (more precisely, the value returned by DEFAULTELLIPSOID) is
%   used.  GEODPROJ defines the projection and gives the restrictions on
%   the allowed ranges of the arguments.  The forward projection is given
%   by EQDAZIM_FWD.
%
%   AZI and RK give metric properties of the projection at (LAT,LON); AZI
%   is the azimuth of the geodesic from the center of projection and RK is
%   the reciprocal of the azimuthal scale.  The scale in the radial
%   direction is 1.
%
%   LAT0, LON0, LAT, LON, AZI are in degrees.  The projected coordinates X,
%   Y are in meters (more precisely the units used for the equatorial
%   radius).  RK is dimensionless.
%
%   Section 14 of
%
%     C. F. F. Karney, Geodesics on an ellipsoid of revolution (2011),
%     http://arxiv.org/abs/1102.1215
%     Errata: http://geographiclib.sf.net/geod-addenda.html#geod-errata
%
%   describes how to use this projection in the determination of maritime
%   boundaries (finding the median line).
%
%   This routine depends on the MATLAB File Exchange package "Geodesics on
%   an ellipsoid of revolution":
%
%     http://www.mathworks.com/matlabcentral/fileexchange/39108
%
%   See also GEODPROJ, EQDAZIM_FWD, GEODRECKON, DEFAULTELLIPSOID.

% Copyright (c) Charles Karney (2012) <charles@karney.com>.
%
% This file was distributed with GeographicLib 1.29.

  if nargin < 4, error('Too few input arguments'), end
  if nargin < 5, ellipsoid = defaultellipsoid; end
  try
    [~] = lat0 + lon0 + x + y;
  catch err
    error('lat0, lon0, x, y have incompatible sizes')
  end

  azi0 = atan2(x, y) / (pi/180);
  s = hypot(x, y);
  [lat, lon, azi, ~, m, ~, ~, sig] = geodreckon(lat0, lon0, s, azi0, ellipsoid);
  rk = m ./ s;
  rk(sig <= 0.01 * sqrt(realmin)) = 1;
end
