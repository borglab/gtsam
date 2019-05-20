function [lat, lon, gam, k] = polarst_inv(isnorth, x, y, ellipsoid)
%POLARST_INV  Inverse polar stereographic projection
%
%   [lat, lon] = POLARST_INV(isnorth, x, y)
%   [lat, lon, gam, k] = POLARST_INV(isnorth, x, y, ellipsoid)
%
%   performs the inverse polar stereographic projection of points (x,y) to
%   (lat,lon) using the north (south) as the center of projection depending
%   on whether isnortp is 1 (0).  These input arguments can be scalars or
%   arrays of equal size.  The ellipsoid vector is of the form [a, e],
%   where a is the equatorial radius in meters, e is the eccentricity.  If
%   ellipsoid is omitted, the WGS84 ellipsoid (more precisely, the value
%   returned by defaultellipsoid) is used.  projdoc defines the projection
%   and gives the restrictions on the allowed ranges of the arguments.  The
%   forward projection is given by polarst_fwd.
%
%   gam and k give metric properties of the projection at (lat,lon); gam is
%   the meridian convergence at the point and k is the scale.
%
%   lat, lon, gam are in degrees.  The projected coordinates x, y are in
%   meters (more precisely the units used for the equatorial radius).  k is
%   dimensionless.
%
%   See also PROJDOC, POLARST_FWD, UTMUPS_FWD, UTMUPS_INV,
%     DEFAULTELLIPSOID, FLAT2ECC.

% Copyright (c) Charles Karney (2015) <charles@karney.com>.

  narginchk(3, 4)
  if nargin < 4, ellipsoid = defaultellipsoid; end
  try
    Z = zeros(size(isnorth + x + y));
  catch
    error('isnorth, x, y have incompatible sizes')
  end
  if length(ellipsoid(:)) ~= 2
    error('ellipsoid must be a vector of size 2')
  end

  a = ellipsoid(1);
  e2 = real(ellipsoid(2)^2);
  e2m = 1 - e2;
  c = sqrt(e2m) * exp(eatanhe(1, e2));

  isnorth = 2 * logical(isnorth) - 1;
  rho = hypot(x, y);
  t = rho / (2 * a / c);
  taup = (1 ./ t - t) / 2;
  tau = tauf(taup, e2);
  lat = atand(tau);
  lat(rho == 0) = 90;
  lat = isnorth .* lat;
  lon = atan2dx(x, -isnorth .* y);
  if nargout > 2
    gam = AngNormalize(isnorth .* lon);
    if nargout > 3
      secphi = hypot(1, tau);
      k = (rho / a) .* secphi .* sqrt(e2m + e2 .* secphi.^-2) + Z;
      k(rho == 0) = 1;
    end
  end
end
