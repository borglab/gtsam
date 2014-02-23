function [x, y, gam, k] = tranmerc_fwd(lat0, lon0, lat, lon, ellipsoid)
%TRANMERC_FWD  Forward transverse Mercator projection
%
%   [X, Y] = TRANMERC_FWD(LAT0, LON0, LAT, LON)
%   [X, Y, GAM, K] = TRANMERC_FWD(LAT0, LON0, LAT, LON, ELLIPSOID)
%
%   performs the forward transverse Mercator projection of points (LAT,LON)
%   to (X,Y) using (LAT0,LON0) as the center of projection.  These input
%   arguments can be scalars or arrays of equal size.  The ELLIPSOID vector
%   is of the form [a, e], where a is the equatorial radius in meters, e is
%   the eccentricity.  If ellipsoid is omitted, the WGS84 ellipsoid (more
%   precisely, the value returned by DEFAULTELLIPSOID) is used.  GEODPROJ
%   defines the projection and gives the restrictions on the allowed ranges
%   of the arguments.  The inverse projection is given by TRANMERC_INV.
%
%   GAM and K give metric properties of the projection at (LAT,LON); GAM is
%   the meridian convergence at the point and K is the scale.
%
%   LAT0, LON0, LAT, LON, GAM are in degrees.  The projected coordinates X,
%   Y are in meters (more precisely the units used for the equatorial
%   radius).  K is dimensionless.
%
%   This implementation of the projection is based on the series method
%   described in
%
%     C. F. F. Karney, Transverse Mercator with an accuracy of a few
%     nanometers, J. Geodesy 85(8), 475-485 (Aug. 2011);
%     Addenda: http://geographiclib.sf.net/tm-addenda.html
%
%   This extends the series given by Krueger (1912) to sixth order in the
%   flattening.  This is a substantially better series than that used by
%   the MATLAB mapping toolbox.  In particular the errors in the projection
%   are less than 5 nanometers withing 3900 km of the central meridian (and
%   less than 1 mm within 7600 km of the central meridian).  The mapping
%   can be continued accurately over the poles to the opposite meridian.
%
%   This routine depends on the MATLAB File Exchange package "Geodesics on
%   an ellipsoid of revolution":
%
%     http://www.mathworks.com/matlabcentral/fileexchange/39108
%
%   See also GEODPROJ, TRANMERC_INV, GEODDISTANCE, DEFAULTELLIPSOID.

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

  degree = pi/180;
  maxpow = 6;

  a = ellipsoid(1);
  f = ecc2flat(ellipsoid(2));
  e2 = f * (2 - f);
  e2m = 1 - e2;
  cc = sqrt(e2m) * exp(e2 * atanhee(1, e2));
  n = f / (2 -f);
  alp = alpf(n);
  b1 = (1 - f) * (A1m1f(n) + 1);
  a1 = b1 * a;

  lon = AngDiff(AngNormalize(lon0), AngNormalize(lon));

  latsign = 1 - 2 * (lat < 0);
  lonsign = 1 - 2 * (lon < 0);
  lon = lon .* lonsign;
  lat = lat .* latsign;
  backside = lon > 90;
  latsign(backside & lat == 0) = -1;
  lon(backside) = 180 - lon(backside);
  phi = lat * degree;
  lam = lon * degree;
  c = max(0, cos(lam));
  tau = tan(phi);
  taup = taupf(tau, e2);
  xip = atan2(taup, c);
  etap = asinh(sin(lam) ./ hypot(taup, c));
  gam = atan(tan(lam) .* taup ./ hypot(1, taup));
  k = sqrt(e2m + e2 * cos(phi).^2) .* hypot(1, tau) ./ hypot(taup, c);
  c = ~(lat ~= 90);
  if any(c)
    xip(c) = pi/2;
    etap(c) = 0;
    gam(c) = lam;
    k = cc;
  end
  c0 = cos(2 * xip); ch0 = cosh(2 * etap);
  s0 = sin(2 * xip); sh0 = sinh(2 * etap);
  ar = 2 * c0 .* ch0; ai = -2 * s0 .* sh0;
  j = maxpow;
  xi0 = Z; yr0 = Z;
  if mod(j, 2)
    xi0 = xi0 + alp(j);
    yr0 = yr0 + 2 * maxpow * alp(j);
    j = j - 1;
  end
  xi1 = Z; eta0 = Z; eta1 = Z;
  yi0 = Z; yr1 = Z; yi1 = Z;
  for j = j : -2 : 1
    xi1  = ar .* xi0 - ai .* eta0 - xi1 + alp(j);
    eta1 = ai .* xi0 + ar .* eta0 - eta1;
    yr1 = ar .* yr0 - ai .* yi0 - yr1 + 2 * j * alp(j);
    yi1 = ai .* yr0 + ar .* yi0 - yi1;
    xi0  = ar .* xi1 - ai .* eta1 - xi0 + alp(j-1);
    eta0 = ai .* xi1 + ar .* eta1 - eta0;
    yr0 = ar .* yr1 - ai .* yi1 - yr0 + 2 * (j-1) * alp(j-1);
    yi0 = ai .* yr1 + ar .* yi1 - yi0;
  end
  ar = ar/2; ai = ai/2;
  yr1 = 1 - yr1 + ar .* yr0 - ai .* yi0;
  yi1 =   - yi1 + ai .* yr0 + ar .* yi0;
  ar = s0 .* ch0; ai = c0 .* sh0;
  xi  = xip  + ar .* xi0 - ai .* eta0;
  eta = etap + ai .* xi0 + ar .* eta0;
  gam = gam - atan2(yi1, yr1);
  k = k .* (b1 * hypot(yr1, yi1));
  gam = gam / degree;
  xi(backside) = pi - xi(backside);
  y = a1 * xi .* latsign;
  x = a1 * eta .* lonsign;
  gam(backside) = 180 - gam(backside);
  gam = gam .* latsign .* lonsign;

  if isscalar(lat0) && lat0 == 0
    y0 = 0;
  else
    [sbet0, cbet0] = SinCosNorm((1-f) * sind(lat0), cosd(lat0));
    y0 = a1 * (atan2(sbet0, cbet0) + ...
               SinCosSeries(true, sbet0, cbet0, C1f(n)));
  end
  y = y - y0;
end

function alp = alpf(n)
  alp = zeros(1,6);
  nx = n^2;

  alp(1) = n*(n*(n*(n*(n*(31564*n-66675)+34440)+47250)-100800)+ ...
              75600)/151200;
  alp(2) = nx*(n*(n*((863232-1983433*n)*n+748608)-1161216)+524160)/ ...
           1935360;
  nx = nx * n;
  alp(3) = nx*(n*(n*(670412*n+406647)-533952)+184464)/725760;
  nx = nx * n;
  alp(4) = nx*(n*(6601661*n-7732800)+2230245)/7257600;
  nx = nx * n;
  alp(5) = (3438171-13675556*n)*nx/7983360;
  nx = nx * n;
  alp(6) = 212378941*nx/319334400;
end

function taup = taupf(tau, e2)
  tau1 = hypot(1, tau);
  sig = sinh( e2 * atanhee(tau ./ tau1, e2) );
  taup = hypot(1, sig) .* tau - sig .* tau1;
  overflow = 1/eps^2;
  c = ~(abs(tau) < overflow);
  taup(c) = tau(c);
end
