function [lat, lon, azi, rk] = gnomonic_inv(lat0, lon0, x, y, ellipsoid)
%GNOMONIC_INV  Inverse ellipsoidal gnomonic projection
%
%   [lat, lon] = GNOMONIC_INV(lat0, lon0, x, y)
%   [lat, lon, azi, rk] = GNOMONIC_INV(lat0, lon0, x, y, ellipsoid)
%
%   performs the inverse ellipsoidal gnomonic projection of points (x,y) to
%   (lat,lon) using (lat0,lon0) as the center of projection.  These input
%   arguments can be scalars or arrays of equal size.  The ellipsoid vector
%   is of the form [a, e], where a is the equatorial radius in meters, e is
%   the eccentricity.  If ellipsoid is omitted, the WGS84 ellipsoid (more
%   precisely, the value returned by defaultellipsoid) is used.  projdoc
%   defines the projection and gives the restrictions on the allowed ranges
%   of the arguments.  The forward projection is given by gnomonic_fwd.
%
%   azi and rk give metric properties of the projection at (lat,lon); azi
%   is the azimuth of the geodesic from the center of projection and rk is
%   the reciprocal of the azimuthal scale.  The scale in the radial
%   direction is 1/rk^2.
%
%   In principle, all finite x and y are allowed.  However, it's possible
%   that the inverse projection fails for very large x and y (when the
%   geographic position is close to the "horizon").  In that case, NaNs are
%   returned for the corresponding output variables.
%
%   lat0, lon0, lat, lon, azi are in degrees.  The projected coordinates x,
%   y are in meters (more precisely the units used for the equatorial
%   radius).  rk is dimensionless.
%
%   The ellipsoidal gnomonic projection is an azimuthal projection about a
%   center point.  All geodesics through the center point are projected
%   into straight lines with the correct azimuth relative to the center
%   point.  In addition all geodesics are pass close to the center point
%   are very nearly straight.  The projection is derived in Section 8 of
%
%     C. F. F. Karney, Algorithms for geodesics,
%     J. Geodesy 87, 43-55 (2013);
%     https://doi.org/10.1007/s00190-012-0578-z
%     Addenda: https://geographiclib.sourceforge.io/geod-addenda.html
%
%   which also includes methods for solving the "intersection" and
%   "interception" problems using the gnomonic projection.
%
%   See also PROJDOC, GNOMONIC_FWD, GEODRECKON, DEFAULTELLIPSOID, FLAT2ECC.

% Copyright (c) Charles Karney (2012-2015) <charles@karney.com>.

  narginchk(4, 5)
  if nargin < 5, ellipsoid = defaultellipsoid; end
  try
    Z = zeros(size(lat0 + lon0 + x + y));
  catch
    error('lat0, lon0, x, y have incompatible sizes')
  end
  if length(ellipsoid(:)) ~= 2
    error('ellipsoid must be a vector of size 2')
  end
  a = ellipsoid(1);
  numit = 10;
  eps1 = a * 0.01 * sqrt(eps);

  lat0 = lat0 + Z; lon0 = lon0 + Z; x = x + Z; y = y + Z;
  azi0 = atan2dx(x, y);
  rho = hypot(x, y);
  s = a * atan(rho / a);
  little = rho <= a;
  rho(~little) = 1 ./ rho(~little);
  g = Z == 0; trip = ~g;
  lat = Z; lon = Z; azi = Z; m = Z; M = Z; ds = Z;
  for k = 1 : numit
    [lat(g), lon(g), azi(g), ~, m(g), M(g)] = ...
        geodreckon(lat0(g), lon0(g), s(g), azi0(g), ellipsoid);
    g = g & ~trip;
    if ~any(g), break, end
    c = little & g;
    ds(c) = (m(c) ./ M(c) - rho(c)) .* M(c).^2;
    c = ~little & g;
    ds(c) = (rho(c) - M(c) ./ m(c)) .* m(c).^2;
    s(g) = s(g) - ds(g);
    trip(g) = ~(abs(ds(g)) >= eps1);
  end
  c = ~trip;
  if any(c)
    lat(c) = nan;
    lon(c) = nan;
    azi(c) = nan;
    M(c) = nan;
  end
  rk = M;
end
