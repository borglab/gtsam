function [x, y, gam, k] = tranmerc_fwd(lat0, lon0, lat, lon, ellipsoid)
%TRANMERC_FWD  Forward transverse Mercator projection
%
%   [x, y] = TRANMERC_FWD(lat0, lon0, lat, lon)
%   [x, y, gam, k] = TRANMERC_FWD(lat0, lon0, lat, lon, ellipsoid)
%
%   performs the forward transverse Mercator projection of points (lat,lon)
%   to (x,y) using (lat0,lon0) as the center of projection.  These input
%   arguments can be scalars or arrays of equal size.  The ellipsoid vector
%   is of the form [a, e], where a is the equatorial radius in meters, e is
%   the eccentricity.  If ellipsoid is omitted, the WGS84 ellipsoid (more
%   precisely, the value returned by defaultellipsoid) is used.  The common
%   case of lat0 = 0 is treated efficiently provided that lat0 is specified
%   as a scalar.  projdoc defines the projection and gives the restrictions
%   on the allowed ranges of the arguments.  The inverse projection is
%   given by tranmerc_inv.  The scale on the central meridian is 1.
%
%   gam and k give metric properties of the projection at (lat,lon); gam is
%   the meridian convergence at the point and k is the scale.
%
%   lat0, lon0, lat, lon, gam are in degrees.  The projected coordinates x,
%   y are in meters (more precisely the units used for the equatorial
%   radius).  k is dimensionless.
%
%   This implementation of the projection is based on the series method
%   described in
%
%     C. F. F. Karney, Transverse Mercator with an accuracy of a few
%     nanometers, J. Geodesy 85(8), 475-485 (Aug. 2011);
%     Addenda: https://geographiclib.sourceforge.io/tm-addenda.html
%
%   This extends the series given by Krueger (1912) to sixth order in the
%   flattening.  This is a substantially better series than that used by
%   the MATLAB mapping toolbox.  In particular the errors in the projection
%   are less than 5 nanometers withing 3900 km of the central meridian (and
%   less than 1 mm within 7600 km of the central meridian).  The mapping
%   can be continued accurately over the poles to the opposite meridian.
%
%   See also PROJDOC, TRANMERC_INV, UTMUPS_FWD, UTMUPS_INV,
%     DEFAULTELLIPSOID, FLAT2ECC.

% Copyright (c) Charles Karney (2012-2017) <charles@karney.com>.

  narginchk(4, 5)
  if nargin < 5, ellipsoid = defaultellipsoid; end
  try
    S = size(lat0 + lon0 + lat + lon);
  catch
    error('lat0, lon0, lat, lon have incompatible sizes')
  end
  if length(ellipsoid(:)) ~= 2
    error('ellipsoid must be a vector of size 2')
  end

  Z = zeros(prod(S),1);
  maxpow = 6;

  a = ellipsoid(1);
  e2 = real(ellipsoid(2)^2);
  f = e2 / (1 + sqrt(1 - e2));
  e2m = 1 - e2;
  cc = sqrt(e2m) * exp(eatanhe(1, e2));
  n = f / (2 -f);
  alp = alpf(n);
  b1 = (1 - f) * (A1m1f(n) + 1);
  a1 = b1 * a;

  lat = LatFix(lat(:)) + Z;
  lon = AngDiff(lon0(:), lon(:)) + Z;

  latsign = 1 - 2 * (lat < 0);
  lonsign = 1 - 2 * (lon < 0);
  lon = lon .* lonsign;
  lat = lat .* latsign;
  backside = lon > 90;
  latsign(backside & lat == 0) = -1;
  lon(backside) = 180 - lon(backside);
  [sphi, cphi] = sincosdx(lat);
  [slam, clam] = sincosdx(lon);
  tau = sphi ./ max(sqrt(realmin), cphi);
  taup = taupf(tau, e2);
  xip = atan2(taup, clam);
  etap = asinh(slam ./ hypot(taup, clam));
  gam = atan2dx(slam .* taup, clam .* hypot(1, taup));
  k = sqrt(e2m + e2 * cphi.^2) .* hypot(1, tau) ./ hypot(taup, clam);
  c = ~(lat ~= 90);
  if any(c)
    xip(c) = pi/2;
    etap(c) = 0;
    gam(c) = lon(c);
    k(c) = cc;
  end
  c0 = cos(2 * xip); ch0 = cosh(2 * etap);
  s0 = sin(2 * xip); sh0 = sinh(2 * etap);
  a = 2 * complex(c0 .* ch0, -s0 .* sh0);
  j = maxpow;
  y0 = complex(Z); y1 = y0;
  z0 = y0; z1 = y0;
  if mod(j, 2)
    y0 = y0 + alp(j);
    z0 = z0 + 2*j * alp(j);
    j = j - 1;
  end
  for j = j : -2 : 1
    y1 = a .* y0 - y1 +       alp(j);
    z1 = a .* z0 - z1 + 2*j * alp(j);
    y0 = a .* y1 - y0 +           alp(j-1);
    z0 = a .* z1 - z0 + 2*(j-1) * alp(j-1);
  end
  a = a/2;
  z1 = 1 - z1 + z0 .* a;
  a = complex(s0 .* ch0, c0 .* sh0);
  y1 = complex(xip, etap) + y0 .* a;
  gam = gam - atan2dx(imag(z1), real(z1));
  k = k .* (b1 * abs(z1));
  xi = real(y1); eta = imag(y1);
  xi(backside) = pi - xi(backside);
  y = a1 * xi .* latsign;
  x = a1 * eta .* lonsign;
  gam(backside) = 180 - gam(backside);
  gam = AngNormalize(gam .* latsign .* lonsign);

  if isscalar(lat0) && lat0 == 0
    y0 = 0;
  else
    [sbet0, cbet0] = sincosdx(LatFix(lat0(:)));
    [sbet0, cbet0] = norm2((1-f) * sbet0, cbet0);
    y0 = a1 * (atan2(sbet0, cbet0) + ...
               SinCosSeries(true, sbet0, cbet0, C1f(n)));
  end
  y = y - y0;

  x = reshape(x, S); y = reshape(y, S);
  gam = reshape(gam, S); k = reshape(k, S);
end

function alp = alpf(n)
  persistent alpcoeff
  if isempty(alpcoeff)
    alpcoeff = [ ...
        31564, -66675, 34440, 47250, -100800, 75600, 151200, ...
        -1983433, 863232, 748608, -1161216, 524160, 1935360, ...
        670412, 406647, -533952, 184464, 725760, ...
        6601661, -7732800, 2230245, 7257600, ...
        -13675556, 3438171, 7983360, ...
        212378941, 319334400, ...
               ];
  end
  maxpow = 6;
  alp = zeros(1, maxpow);
  o = 1;
  d = n;
  for l = 1 : maxpow
    m = maxpow - l;
    alp(l) = d * polyval(alpcoeff(o : o + m), n) / alpcoeff(o + m + 1);
    o = o + m + 2;
    d = d * n;
  end
end
