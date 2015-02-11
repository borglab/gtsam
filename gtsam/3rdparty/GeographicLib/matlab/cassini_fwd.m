function [x, y, azi, rk] = cassini_fwd(lat0, lon0, lat, lon, ellipsoid)
%CASSINI_FWD  Forward Cassini-Soldner projection
%
%   [X, Y] = CASSINI_FWD(LAT0, LON0, LAT, LON)
%   [X, Y, AZI, RK] = CASSINI_FWD(LAT0, LON0, LAT, LON, ELLIPSOID)
%
%   performs the forward Cassini-Soldner projection of points (LAT,LON) to
%   (X,Y) using (LAT0,LON0) as the center of projection.  These input
%   arguments can be scalars or arrays of equal size.  The ELLIPSOID vector
%   is of the form [a, e], where a is the equatorial radius in meters, e is
%   the eccentricity.  If ellipsoid is omitted, the WGS84 ellipsoid (more
%   precisely, the value returned by DEFAULTELLIPSOID) is used.  GEODPROJ
%   defines the projection and gives the restrictions on the allowed ranges
%   of the arguments.  The inverse projection is given by CASSINI_INV.
%
%   AZI and RK give metric properties of the projection at (LAT,LON); AZI
%   is the azimuth of the easting (X) direction and RK is the reciprocal of
%   the northing (Y) scale.  The scale in the easting direction is 1.
%
%   LAT0, LON0, LAT, LON, AZI are in degrees.  The projected coordinates X,
%   Y are in meters (more precisely the units used for the equatorial
%   radius).  RK is dimensionless.
%
%   This routine depends on the MATLAB File Exchange package "Geodesics on
%   an ellipsoid of revolution":
%
%     http://www.mathworks.com/matlabcentral/fileexchange/39108
%
%   See also GEODPROJ, CASSINI_INV, GEODDISTANCE, DEFAULTELLIPSOID.

% Copyright (c) Charles Karney (2012) <charles@karney.com>.
%
% This file was distributed with GeographicLib 1.29.

  if nargin < 4, error('Too few input arguments'), end
  if nargin < 5, ellipsoid = defaultellipsoid; end
  try
    Z = lat0 + lon0 + lat + lon;
    Z = zeros(size(Z));
  catch err
    error('lat0, lon0, lat, lon have incompatible sizes')
  end
  if length(ellipsoid(:)) ~= 2
    error('ellipsoid must be a vector of size 2')
  end

  tiny = sqrt(realmin);
  degree = pi/180;
  f = ecc2flat(ellipsoid(2));
  lat = AngRound(lat);
  dlon = AngDiff(AngNormalize(lon0), AngNormalize(lon)) + Z;
  [s12, azi1, azi2, ~, ~, ~, ~, sig12] = ...
      geoddistance(lat, -abs(dlon), lat, abs(dlon), ellipsoid);
  c = sig12 < 100 * tiny;
  sig12(c) = 0;
  s12(c) = 0;
  sig12 = 0.5 * sig12;
  s12 = 0.5 * s12;
  c = s12 == 0;
  da = (azi2 - azi2)/2;
  s = abs(dlon) <= 90;
  azi1(c & s) = 90 - da(c & s);
  azi2(c & s) = 90 + da(c & s);
  s = ~s;
  azi1(c & s) = -90 - da(c & s);
  azi2(c & s) = -90 + da(c & s);
  c = dlon < 0;
  azi2(c) = azi1(c);
  s12(c) = -s12(c);
  sig12(c) = -sig12(c);
  x = s12;
  azi = AngNormalize(azi2);
  [~, ~, ~, ~, ~, ~, rk] = ...
      geodreckon(lat, dlon, -sig12, azi, ellipsoid, true);
  [sbet , cbet ] = SinCosNorm((1-f) * sind(lat ), cosd(lat ));
  [sbet0, cbet0] = SinCosNorm((1-f) * sind(lat0), cosd(lat0));
  alp = azi * degree;
  salp = sin(alp); salp(alp == -180) = 0;
  calp = cos(alp); calp(abs(alp) == 90) = 0;
  salp0 = salp .* cbet;
  calp0 = hypot(calp, salp .* sbet);
  sbet1 = calp0;
  c = lat + Z >= 0;
  sbet1(~c) = -sbet1(~c);
  cbet1 = abs(salp0);
  c = abs(dlon) <= 90;
  cbet1(~c) = -cbet1(~c);
  sbet01 = sbet1 .* cbet0 - cbet1 .* sbet0;
  cbet01 = cbet1 .* cbet0 + sbet1 .* sbet0;
  sig01 = atan2(sbet01, cbet01) / degree;
  [~, ~, ~, ~, ~, ~, ~, y] = geodreckon(lat0, 0, sig01, 0, ellipsoid, true);
end
