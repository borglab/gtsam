function [x, y, gam, k] = polarst_fwd(isnorth, lat, lon, ellipsoid)
%POLARST_FWD  Forward polar stereographic projection
%
%   [x, y] = POLARST_FWD(isnorth, lat, lon)
%   [x, y, gam, k] = POLARST_FWD(isnorth, lat, lon, ellipsoid)
%
%   performs the forward polar stereographic projection of points (lat,lon)
%   to (x,y) using the north (south) as the center of projection depending
%   on whether isnortp is 1 (0).  These input arguments can be scalars or
%   arrays of equal size.  The ellipsoid vector is of the form [a, e],
%   where a is the equatorial radius in meters, e is the eccentricity.  If
%   ellipsoid is omitted, the WGS84 ellipsoid (more precisely, the value
%   returned by defaultellipsoid) is used.  projdoc defines the projection
%   and gives the restrictions on the allowed ranges of the arguments.  The
%   inverse projection is given by polarst_inv.
%
%   gam and k give metric properties of the projection at (lat,lon); gam is
%   the meridian convergence at the point and k is the scale.
%
%   lat, lon, gam are in degrees.  The projected coordinates x, y are in
%   meters (more precisely the units used for the equatorial radius).  k is
%   dimensionless.
%
%   See also PROJDOC, POLARST_INV, UTMUPS_FWD, UTMUPS_INV,
%     DEFAULTELLIPSOID, FLAT2ECC.

% Copyright (c) Charles Karney (2015) <charles@karney.com>.

  narginchk(3, 4)
  if nargin < 4, ellipsoid = defaultellipsoid; end
  try
    Z = zeros(size(isnorth + lat + lon));
  catch
    error('isnorth, lat, lon have incompatible sizes')
  end
  if length(ellipsoid(:)) ~= 2
    error('ellipsoid must be a vector of size 2')
  end

  overflow = 1/eps^2;
  a = ellipsoid(1);
  e2 = real(ellipsoid(2)^2);
  e2m = 1 - e2;
  c = sqrt(e2m) * exp(eatanhe(1, e2));

  isnorth = 2 * logical(isnorth) - 1;
  lat = LatFix(lat) .* isnorth;
  tau = tand(lat); tau(abs(lat) == 90) = sign(lat(abs(lat) == 90)) * overflow;
  taup = taupf(tau, e2);
  rho = hypot(1, taup) + abs(taup);
  rho(taup >= 0) = cvmgt(1./rho(taup >= 0), 0, lat(taup >= 0) ~= 90);
  rho = rho * (2 * a / c);
  [x, y] = sincosdx(lon);
  x = rho .* x;
  y = -isnorth .* rho .* y;
  if nargout > 2
    gam = AngNormalize(isnorth .* lon) + Z;
    if nargout > 3
      secphi = hypot(1, tau);
      k = (rho / a) .* secphi .* sqrt(e2m + e2 .* secphi.^-2) + Z;
      k(lat == 90) = 1;
    end
  end
end
