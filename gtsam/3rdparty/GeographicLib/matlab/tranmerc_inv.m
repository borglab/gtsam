function [lat, lon, gam, k] = tranmerc_inv(lat0, lon0, x, y, ellipsoid)
%TRANMERC_INV  Inverse transverse Mercator projection
%
%   [LAT, LON] = TRANMERC_INV(LAT0, LON0, X, Y)
%   [LAT, LON, GAM, K] = TRANMERC_INV(LAT0, LON0, X, Y, ELLIPSOID)
%
%   performs the inverse transverse Mercator projection of points (X,Y) to
%   (LAT,LON) using (LAT0,LON0) as the center of projection.  These input
%   arguments can be scalars or arrays of equal size.  The ELLIPSOID vector
%   is of the form [a, e], where a is the equatorial radius in meters, e is
%   the eccentricity.  If ellipsoid is omitted, the WGS84 ellipsoid (more
%   precisely, the value returned by DEFAULTELLIPSOID) is used.  GEODPROJ
%   defines the projection and gives the restrictions on the allowed ranges
%   of the arguments.  The forward projection is given by TRANMERC_FWD.
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
%   See also GEODPROJ, TRANMERC_FWD, GEODRECKON, DEFAULTELLIPSOID.

% Copyright (c) Charles Karney (2012) <charles@karney.com>.
%
% This file was distributed with GeographicLib 1.29.

  if nargin < 4, error('Too few input arguments'), end
  if nargin < 5, ellipsoid = defaultellipsoid; end
  try
    Z = lat0 + lon0 + x + y;
    Z = zeros(size(Z));
  catch err
    error('lat0, lon0, x, y have incompatible sizes')
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
  bet = betf(n);
  b1 = (1 - f) * (A1m1f(n) + 1);
  a1 = b1 * a;

  if isscalar(lat0) && lat0 == 0
    y0 = 0;
  else
    [sbet0, cbet0] = SinCosNorm((1-f) * sind(lat0), cosd(lat0));
    y0 = a1 * (atan2(sbet0, cbet0) + ...
               SinCosSeries(true, sbet0, cbet0, C1f(n)));
  end
  y = y + y0;

  xi = y / a1;
  eta = x / a1;
  xisign = 1 - 2 * (xi < 0 );
  etasign = 1 - 2 * (eta < 0 );
  xi = xi .* xisign;
  eta = eta .* etasign;
  backside = xi > pi/2;
  xi(backside) = pi - xi(backside);

  c0 = cos(2 * xi); ch0 = cosh(2 * eta);
  s0 = sin(2 * xi); sh0 = sinh(2 * eta);
  ar = 2 * c0 .* ch0; ai = -2 * s0 .* sh0;
  j = maxpow;
  xip0 = Z; yr0 = Z;
  if mod(j, 2)
    xip0 = xip0 + bet(j);
    yr0 = yr0 - 2 * maxpow * bet(j);
    j = j - 1;
  end
  xip1 = Z; etap0 = Z; etap1 = Z;
  yi0 = Z; yr1 = Z; yi1 = Z;
  for j = j : -2 : 1
    xip1  = ar .* xip0 - ai .* etap0 - xip1 - bet(j);
    etap1 = ai .* xip0 + ar .* etap0 - etap1;
    yr1 = ar .* yr0 - ai .* yi0 - yr1 - 2 * j * bet(j);
    yi1 = ai .* yr0 + ar .* yi0 - yi1;
    xip0  = ar .* xip1 - ai .* etap1 - xip0 - bet(j-1);
    etap0 = ai .* xip1 + ar .* etap1 - etap0;
    yr0 = ar .* yr1 - ai .* yi1 - yr0 - 2 * (j-1) * bet(j-1);
    yi0 = ai .* yr1 + ar .* yi1 - yi0;
  end
  ar = ar/2; ai = ai/2;
  yr1 = 1 - yr1 + ar .* yr0 - ai .* yi0;
  yi1 =   - yi1 + ai .* yr0 + ar .* yi0;
  ar = s0 .* ch0; ai = c0 .* sh0;
  xip  = xi  + ar .* xip0 - ai .* etap0;
  etap = eta + ai .* xip0 + ar .* etap0;
  gam = atan2(yi1, yr1);
  k = b1 ./ hypot(yr1, yi1);
  s = sinh(etap);
  c = max(0, cos(xip));
  r = hypot(s, c);
  lam = atan2(s, c);
  taup = sin(xip)./r;
  tau = tauf(taup, e2);
  phi = atan(tau);
  gam = gam + atan(tan(xip) .* tanh(etap));
  c = r ~= 0;
  k(c) = k(c) .* sqrt(e2m + e2 * cos(phi(c)).^2) .* ...
         hypot(1, tau(c)) .* r(c);
  c = ~c;
  if any(c)
    phi(c) = pi/2;
    lam(c) = 0;
    k(c) = k(c) * cc;
  end
  lat = phi / degree .* xisign;
  lon = lam / degree;
  lon(backside) = 180 - lon(backside);
  lon = lon .* etasign;
  lon = AngNormalize(lon + AngNormalize(lon0));
  gam = gam/degree;
  gam(backside) = 180 - gam(backside);
  gam = gam .* xisign .* etasign;
end

function bet = betf(n)
  bet = zeros(1,6);
  nx = n^2;
  bet(1) = n*(n*(n*(n*(n*(384796*n-382725)-6720)+932400)-1612800)+ ...
              1209600)/2419200;
  bet(2) = nx*(n*(n*((1695744-1118711*n)*n-1174656)+258048)+80640)/ ...
           3870720;
  nx = nx * n;
  bet(3) = nx*(n*(n*(22276*n-16929)-15984)+12852)/362880;
  nx = nx * n;
  bet(4) = nx*((-830251*n-158400)*n+197865)/7257600;
  nx = nx * n;
  bet(5) = (453717-435388*n)*nx/15966720;
  nx = nx * n;
  bet(6) = 20648693*nx/638668800;
end

function tau = tauf(taup, e2)
  overflow = 1/eps^2;
  tol = 0.1 * sqrt(eps);
  numit = 5;
  e2m = 1 - e2;
  tau = taup / e2m;
  stol = tol * max(1, abs(taup));
  g = ~(abs(taup) < overflow);
  tau(g) = taup(g);
  g = ~g;
  for i = 1 : numit
    if ~any(g), break, end
    tau1 = hypot(1, tau);
    sig = sinh(e2 * atanhee( tau ./ tau1, e2 ) );
    taupa = hypot(1, sig) .* tau - sig .* tau1;
    dtau = (taup - taupa) .* (1 + e2m .* tau.^2) ./ ...
           (e2m * tau1 .* hypot(1, taupa));
    tau(g) = tau(g) + dtau(g);
    g = g & abs(dtau) >= stol;
  end
end
