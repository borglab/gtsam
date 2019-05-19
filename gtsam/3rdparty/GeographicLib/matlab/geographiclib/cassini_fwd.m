function [x, y, azi, rk] = cassini_fwd(lat0, lon0, lat, lon, ellipsoid)
%CASSINI_FWD  Forward Cassini-Soldner projection
%
%   [x, y] = CASSINI_FWD(lat0, lon0, lat, lon)
%   [x, y, azi, rk] = CASSINI_FWD(lat0, lon0, lat, lon, ellipsoid)
%
%   performs the forward Cassini-Soldner projection of points (lat,lon) to
%   (x,y) using (lat0,lon0) as the center of projection.  These input
%   arguments can be scalars or arrays of equal size.  The ellipsoid vector
%   is of the form [a, e], where a is the equatorial radius in meters, e is
%   the eccentricity.  If ellipsoid is omitted, the WGS84 ellipsoid (more
%   precisely, the value returned by defaultellipsoid) is used.  projdoc
%   defines the projection and gives the restrictions on the allowed ranges
%   of the arguments.  The inverse projection is given by cassini_inv.
%
%   azi and rk give metric properties of the projection at (lat,lon); azi
%   is the azimuth of the easting (x) direction and rk is the reciprocal of
%   the northing (y) scale.  The scale in the easting direction is 1.
%
%   lat0, lon0, lat, lon, azi are in degrees.  The projected coordinates x,
%   y are in meters (more precisely the units used for the equatorial
%   radius).  rk is dimensionless.
%
%   See also PROJDOC, CASSINI_INV, GEODDISTANCE, DEFAULTELLIPSOID,
%     FLAT2ECC.

% Copyright (c) Charles Karney (2012-2015) <charles@karney.com>.

  narginchk(4, 5)
  if nargin < 5, ellipsoid = defaultellipsoid; end
  try
    Z = zeros(size(lat0 + lon0 + lat + lon));
  catch
    error('lat0, lon0, lat, lon have incompatible sizes')
  end
  if length(ellipsoid(:)) ~= 2
    error('ellipsoid must be a vector of size 2')
  end

  degree = pi/180;
  f = ecc2flat(ellipsoid(2));
  dlon = AngDiff(lon0, lon) + Z;
  [s12, azi1, azi2, ~, ~, ~, ~, sig12] = ...
      geoddistance(lat, -abs(dlon), lat, abs(dlon), ellipsoid);
  sig12 = 0.5 * sig12;
  s12 = 0.5 * s12;
  c = s12 == 0;
  da = AngDiff(azi1, azi2)/2;
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
      geodreckon(lat, dlon, -sig12, azi, ellipsoid, 1);
  [sbet, cbet] = sincosdx(lat);
  [sbet, cbet] = norm2((1-f) * sbet, cbet);
  [sbet0, cbet0] = sincosdx(lat0);
  [sbet0, cbet0] = norm2((1-f) * sbet0, cbet0);
  [salp, calp] = sincosdx(azi);
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
  [~, ~, ~, ~, ~, ~, ~, y] = geodreckon(lat0, 0, sig01, 0, ellipsoid, 1);
end
