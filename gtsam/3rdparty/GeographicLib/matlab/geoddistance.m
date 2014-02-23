function [s12, azi1, azi2, S12, m12, M12, M21, a12] = geoddistance ...
      (lat1, lon1, lat2, lon2, ellipsoid)
%GEODDISTANCE  Distance between points on an ellipsoid
%
%   [s12, azi1, azi2] = GEODDISTANCE(lat1, lon1, lat2, lon2)
%   [s12, azi1, azi2, S12, m12, M12, M21, a12] =
%      GEODDISTANCE(lat1, lon1, lat2, lon2, ellipsoid)
%
%   solves the inverse geodesic problem of finding of length and azimuths
%   of the shortest geodesic between points specified by lat1, lon1, lat2,
%   lon2.  The input latitudes and longitudes, lat1, lon1, lat2, lon2, can
%   be scalars or arrays of equal size and must be expressed in degrees.
%   The ellipsoid vector is of the form [a, e], where a is the equatorial
%   radius in meters, e is the eccentricity.  If ellipsoid is omitted, the
%   WGS84 ellipsoid (more precisely, the value returned by
%   DEFAULTELLIPSOID) is used.  The output s12 is the distance in meters
%   and azi1 and azi2 are the forward azimuths at the end points in
%   degrees.  The other optional outputs, S12, m12, M12, M21, a12 are
%   documented in GEODDOC.  GEODDOC also gives the restrictions on the
%   allowed ranges of the arguments.
%
%   When given a combination of scalar and array inputs, the scalar inputs
%   are automatically expanded to match the size of the arrays.
%
%   This is an implementation of the algorithm given in
%
%     C. F. F. Karney, Algorithms for geodesics,
%     J. Geodesy 87, 43-55 (2013);
%     http://dx.doi.org/10.1007/s00190-012-0578-z
%     Addenda: http://geographiclib.sf.net/geod-addenda.html
%
%   This function duplicates some of the functionality of the DISTANCE
%   function in the MATLAB mapping toolbox.  Differences are
%
%     * When the ellipsoid argument is omitted, use the WGS84 ellipsoid.
%     * The routines work for prolate (as well as oblate) ellipsoids.
%     * The azimuth at the second end point azi2 is returned.
%     * The solution is accurate to round off for abs(e) < 0.2.
%     * The algorithm converges for all pairs of input points.
%     * Additional properties of the geodesic are calcuated.
%
%   See also GEODDOC, GEODRECKON, GEODAREA, GEODESICINVERSE,
%     DEFAULTELLIPSOID.

% Copyright (c) Charles Karney (2012, 2013) <charles@karney.com>.
%
% This file was distributed with GeographicLib 1.31.
%
% This is a straightforward transcription of the C++ implementation in
% GeographicLib and the C++ source should be consulted for additional
% documentation.  This is a vector implementation and the results returned
% with array arguments are identical to those obtained with multiple calls
% with scalar arguments.  The biggest change was to eliminate the branching
% to allow a vectorized solution.

  if nargin < 4, error('Too few input arguments'), end
  if nargin < 5, ellipsoid = defaultellipsoid; end
  try
    Z = lat1 + lon1 + lat2 + lon2;
    S = size(Z);
    Z = zeros(S);
    lat1 = lat1 + Z; lon1 = lon1 + Z;
    lat2 = lat2 + Z; lon2 = lon2 + Z;
    Z = Z(:);
  catch err
    error('lat1, lon1, s12, azi1 have incompatible sizes')
  end
  if length(ellipsoid(:)) ~= 2
    error('ellipsoid must be a vector of size 2')
  end

  degree = pi/180;
  tiny = sqrt(realmin);
  tol0 = eps;
  tolb = eps * sqrt(eps);
  maxit1 = 20;
  maxit2 = maxit1 + (-log2(eps) + 1) + 10;

  a = ellipsoid(1);
  e2 = ellipsoid(2)^2;
  f = e2 / (1 + sqrt(1 - e2));

  f1 = 1 - f;
  ep2 = e2 / (1 - e2);
  n = f / (2 - f);
  b = a * f1;

  areap = nargout >= 4;
  scalp = nargout >= 6;

  A3x = A3coeff(n);
  C3x = C3coeff(n);

  lon12 = AngDiff(AngNormalize(lon1(:)), AngNormalize(lon2(:)));
  lon12 = AngRound(lon12);
  lonsign = 2 * (lon12 >= 0) - 1;
  lon12 = lonsign .* lon12;
  lat1 = AngRound(lat1(:));
  lat2 = AngRound(lat2(:));
  swapp = 2 * (abs(lat1) >= abs(lat2)) - 1;
  lonsign(swapp < 0) = - lonsign(swapp < 0);
  [lat1(swapp < 0), lat2(swapp < 0)] = swap(lat1(swapp < 0), lat2(swapp < 0));

  latsign = 2 * (lat1 < 0) - 1;
  lat1 = latsign .* lat1;
  lat2 = latsign .* lat2;

  phi = lat1 * degree;
  sbet1 = f1 * sin(phi); cbet1 = cos(phi); cbet1(lat1 == -90) = tiny;
  [sbet1, cbet1] = SinCosNorm(sbet1, cbet1);

  phi = lat2 * degree;
  sbet2 = f1 * sin(phi); cbet2 = cos(phi); cbet2(abs(lat2) == 90) = tiny;
  [sbet2, cbet2] = SinCosNorm(sbet2, cbet2);

  c = cbet1 < -sbet1 & cbet2 == cbet1;
  sbet2(c) = (2 * (sbet2(c) < 0) - 1) .* sbet1(c);
  c = ~(cbet1 < -sbet1) & abs(sbet2) == - sbet1;
  cbet2(c) = cbet1(c);

  dn1 = sqrt(1 + ep2 * sbet1.^2);
  dn2 = sqrt(1 + ep2 * sbet2.^2);
  lam12 = lon12 * degree;
  slam12 = sin(lam12); slam12(lon12 == 180) = 0; clam12 = cos(lam12);

  sig12 = Z; ssig1 = Z; csig1 = Z; ssig2 = Z; csig2 = Z;
  calp1 = Z; salp1 = Z; calp2 = Z; salp2 = Z;
  s12 = Z; m12 = Z; M12 = Z; M21 = Z; omg12 = Z;

  m = lat1 == -90 | slam12 == 0;

  if any(m)
    calp1(m) = clam12(m); salp1(m) = slam12(m);
    calp2(m) = 1; salp2(m) = 0;

    ssig1(m) = sbet1(m); csig1(m) = calp1(m) .* cbet1(m);
    ssig2(m) = sbet2(m); csig2(m) = calp2(m) .* cbet2(m);

    sig12(m) = atan2(max(csig1(m) .* ssig2(m) - ssig1(m) .* csig2(m), 0), ...
                     csig1(m) .* csig2(m) + ssig1(m) .* ssig2(m));

    [s12(m), m12(m), ~, M12(m), M21(m)] = ...
        Lengths(n, sig12(m), ...
                ssig1(m), csig1(m), dn1(m), ssig2(m), csig2(m), dn2(m), ...
                cbet1(m), cbet2(m), scalp, ep2);
    m = m & (sig12 < 1 | m12 >= 0);
    m12(m) = m12(m) * b;
    s12(m) = s12(m) * b;
  end

  eq = ~m & sbet1 == 0;
  if f > 0
    eq = eq & lam12 < pi - f * pi;
  end
  calp1(eq) = 0; calp2(eq) = 0; salp1(eq) = 1; salp2(eq) = 1;
  s12(eq) = a * lam12(eq); sig12(eq) = lam12(eq) / f1; omg12(eq) = sig12(eq);
  m12(eq) = b * sin(omg12(eq)); M12(eq) = cos(omg12(eq)); M21(eq) = M12(eq);

  g = ~eq & ~m;

  dnm = Z;
  [sig12(g), salp1(g), calp1(g), salp2(g), calp2(g), dnm(g)] = ...
      InverseStart(sbet1(g), cbet1(g), dn1(g), sbet2(g), cbet2(g), dn2(g), ...
                   lam12(g), f, A3x);

  s = g & sig12 >= 0;
  s12(s) = b * sig12(s) .* dnm(s);
  m12(s) = b * dnm(s).^2 .* sin(sig12(s) ./ dnm(s));
  if scalp
    M12(s) = cos(sig12(s) ./ dnm(s)); M21(s) = M12(s);
  end
  omg12(s) = lam12(s) ./ (f1 * dnm(s));

  g = g & sig12 < 0;

  salp1a = Z + tiny; calp1a = Z + 1;
  salp1b = Z + tiny; calp1b = Z - 1;
  ssig1 = Z; csig1 = Z; ssig2 = Z; csig2 = Z;
  epsi = Z; v = Z; dv = Z;
  numit = Z;
  tripn = Z > 0;
  tripb = tripn;
  gsave = g;
  for k = 0 : maxit2 - 1
    if k == 0 && ~any(g), break, end
    numit(g) = k;
    [v(g), dv(g), ...
     salp2(g), calp2(g), sig12(g), ...
     ssig1(g), csig1(g), ssig2(g), csig2(g), epsi(g), omg12(g)] = ...
        Lambda12(sbet1(g), cbet1(g), dn1(g), ...
                 sbet2(g), cbet2(g), dn2(g), ...
                 salp1(g), calp1(g), f, A3x, C3x);
    v = v - lam12;
    g = g & ~(tripb | ~(abs(v) >= ((tripn * 6) + 2) * tol0));
    if ~any(g), break, end

    c = g & v > 0;
    if k <= maxit1
      c = c & calp1 ./ salp1 > calp1b ./ salp1b;
    end
    salp1b(c) = salp1(c); calp1b(c) = calp1(c);

    c = g & v < 0;
    if k <= maxit1
      c = c & calp1 ./ salp1 < calp1a ./ salp1a;
    end
    salp1a(c) = salp1(c); calp1a(c) = calp1(c);

    if k == maxit1, tripn(g) = false; end
    if k < maxit1
      dalp1 = -v ./ dv;
      sdalp1 = sin(dalp1); cdalp1 = cos(dalp1);
      nsalp1 = salp1 .* cdalp1 + calp1 .* sdalp1;
      calp1(g) = calp1(g) .* cdalp1(g) - salp1(g) .* sdalp1(g);
      salp1(g) = nsalp1(g);
      tripn = g & abs(v) <= 16 * tol0;
      c = g & ~(dv > 0 & nsalp1 > 0 & abs(dalp1) < pi);
      tripn(c) = false;
    else
      c = g;
    end

    salp1(c) = (salp1a(c) + salp1b(c))/2;
    calp1(c) = (calp1a(c) + calp1b(c))/2;
    [salp1(g), calp1(g)] = SinCosNorm(salp1(g), calp1(g));
    tripb(c) = (abs(salp1a(c) - salp1(c)) + (calp1a(c) - calp1(c)) < tolb | ...
                abs(salp1(c) - salp1b(c)) + (calp1(c) - calp1b(c)) < tolb);
  end

  g = gsave;
  [s12(g), m12(g), ~, M12(g), M21(g)] = ...
      Lengths(epsi(g), sig12(g), ...
              ssig1(g), csig1(g), dn1(g), ssig2(g), csig2(g), dn2(g), ...
              cbet1(g), cbet2(g), scalp, ep2);

  m12(g) = m12(g) * b;
  s12(g) = s12(g) * b;
  omg12(g) = lam12(g) - omg12(g);

  s12 = 0 + s12;

  if areap
    salp0 = salp1 .* cbet1; calp0 = hypot(calp1, salp1 .* sbet1);
    ssig1 = sbet1; csig1 = calp1 .* cbet1;
    ssig2 = sbet2; csig2 = calp2 .* cbet2;
    k2 = calp0.^2 * ep2;
    epsi = k2 ./ (2 * (1 + sqrt(1 + k2)) + k2);
    A4 = (a^2 * e2) * calp0 .* salp0;
    [ssig1, csig1] = SinCosNorm(ssig1, csig1);
    [ssig2, csig2] = SinCosNorm(ssig2, csig2);

    C4x = C4coeff(n);
    C4a = C4f(epsi, C4x);
    B41 = SinCosSeries(false, ssig1, csig1, C4a);
    B42 = SinCosSeries(false, ssig2, csig2, C4a);
    S12 = A4 .* (B42 - B41);
    S12(calp0 == 0 | salp0 == 0) = 0;

    l = ~m & omg12 < 0.75 * pi & sbet2 - sbet1 < 1.75;
    alp12 = Z;
    somg12 = sin(omg12(l)); domg12 = 1 + cos(omg12(l));
    dbet1 = 1 + cbet1(l); dbet2 = 1 + cbet2(l);
    alp12(l) = 2 * atan2(somg12 .* (sbet1(l) .* dbet2 + sbet2(l) .* dbet1), ...
                         domg12 .* (sbet1(l) .* sbet2(l) + dbet1 .* dbet2));
    l = ~l;
    salp12 = salp2(l) .* calp1(l) - calp2(l) .* salp1(l);
    calp12 = calp2(l) .* calp1(l) + salp2(l) .* salp1(l);
    s = salp12 == 0 & calp12 < 0;
    salp12(s) = tiny * calp1(s); calp12(s) = -1;
    alp12(l) = atan2(salp12, calp12);
    c2 = (a^2 + b^2 * atanhee(1, e2)) / 2;
    S12 = 0 + swapp .* lonsign .* latsign .* (S12 + c2 * alp12);
  end

  [salp1(swapp<0), salp2(swapp<0)] = swap(salp1(swapp<0), salp2(swapp<0));
  [calp1(swapp<0), calp2(swapp<0)] = swap(calp1(swapp<0), calp2(swapp<0));
  if scalp
    [M12(swapp<0), M21(swapp<0)] = swap(M12(swapp<0), M21(swapp<0));
  end
  salp1 = salp1 .* swapp .* lonsign; calp1 = calp1 .* swapp .* latsign;
  salp2 = salp2 .* swapp .* lonsign; calp2 = calp2 .* swapp .* latsign;

  azi1 = 0 - atan2(-salp1, calp1) / degree;
  azi2 = 0 - atan2(-salp2, calp2) / degree;
  a12 = sig12 / degree;

  s12 = reshape(s12, S); azi1 = reshape(azi1, S); azi2 = reshape(azi2, S);
  m12 = reshape(m12, S); M12 = reshape(M12, S); M21 = reshape(M21, S);
  a12 = reshape(a12, S);
  if (areap)
    S12 = reshape(S12, S);
  end
end

function [sig12, salp1, calp1, salp2, calp2, dnm] = ...
      InverseStart(sbet1, cbet1, dn1, sbet2, cbet2, dn2, lam12, f, A3x)
%INVERSESTART  Compute a starting point for Newton's method

  N = length(sbet1);
  f1 = 1 - f;
  e2 = f * (2 - f);
  ep2 = e2 / (1 - e2);
  n = f / (2 - f);
  tol0 = eps;
  tol1 = 200 * tol0;
  tol2 = sqrt(eps);
  etol2 = 0.1 * tol2 / sqrt( max(0.001, abs(f)) * min(1, 1 - f/2) / 2 );
  xthresh = 1000 * tol2;

  sig12 = -ones(N, 1); salp2 = NaN(N, 1); calp2 = NaN(N, 1);
  sbet12 = sbet2 .* cbet1 - cbet2 .* sbet1;
  cbet12 = cbet2 .* cbet1 + sbet2 .* sbet1;
  sbet12a = sbet2 .* cbet1 + cbet2 .* sbet1;
  s = cbet12 >= 0 & sbet12 < 0.5 & cbet2 .* lam12 < 0.5;
  omg12 = lam12;
  dnm = NaN(N, 1);
  sbetm2 = (sbet1(s) + sbet2(s)).^2;
  sbetm2 = sbetm2 ./ (sbetm2 + (cbet1(s) + cbet2(s)).^2);
  dnm(s) = sqrt(1 + ep2 * sbetm2);
  omg12(s) = omg12(s) ./ (f1 * dnm(s));
  somg12 = sin(omg12); comg12 = cos(omg12);

  salp1 = cbet2 .* somg12;
  t = cbet2 .* sbet1 .* somg12.^2;
  calp1 = cvmgt(sbet12  + t ./ (1 + comg12), ...
                sbet12a - t ./ (1 - comg12), ...
                comg12 >= 0);

  ssig12 = hypot(salp1, calp1);
  csig12 = sbet1 .* sbet2 + cbet1 .* cbet2 .* comg12;

  s = s & ssig12 < etol2;
  salp2(s) = cbet1(s) .* somg12(s);
  calp2(s) = somg12(s).^2 ./ (1 + comg12(s));
  calp2(s & comg12 < 0) = 1 - comg12(s & comg12 < 0);
  calp2(s) = sbet12(s) - cbet1(s) .* sbet2(s) .* calp2(s);
  [salp2, calp2] = SinCosNorm(salp2, calp2);
  sig12(s) = atan2(ssig12(s), csig12(s));

  s = ~(s | abs(n) > 0.1 | csig12 >= 0 | ssig12 >= 6 * abs(n) * pi * cbet1.^2);

  if any(s)
    if f >= 0
      k2 = sbet1(s).^2 * ep2;
      epsi = k2 ./ (2 * (1 + sqrt(1 + k2)) + k2);
      lamscale = f * cbet1(s) .* A3f(epsi, A3x) * pi;
      betscale = lamscale .* cbet1(s);
      x = (lam12(s) - pi) ./ lamscale;
      y = sbet12a(s) ./ betscale;
    else
      cbet12a = cbet2(s) .* cbet1(s) - sbet2(s) .* sbet1(s);
      bet12a = atan2(sbet12a(s), cbet12a);
      [~, m12b, m0] = ...
          Lengths(n, pi + bet12a, ...
                  sbet1(s), -cbet1(s), dn1(s), sbet2(s), cbet2(s), dn2(s), ...
                  cbet1(s), cbet2(s), false);
      x = -1 + m12b ./ (cbet1(s) .* cbet2(s) .* m0 * pi);
      betscale = cvmgt(sbet12a(s) ./ x, - f * cbet1(s).^2 * pi, x < -0.01);
      lamscale = betscale ./ cbet1(s);
      y = (lam12(s) - pi) ./ lamscale;
    end
    k = Astroid(x, y);
    if f >= 0
      omg12a = -x .* k ./ (1 + k);
    else
      omg12a = -y .* (1 + k) ./ k;
    end
    omg12a = lamscale .* omg12a;
    somg12 = sin(omg12a); comg12 = -cos(omg12a);
    salp1(s) = cbet2(s) .* somg12;
    calp1(s) = sbet12a(s) - cbet2(s) .* sbet1(s) .* somg12.^2 ./ (1 - comg12);

    str = y > -tol1 & x > -1 - xthresh;
    if any(str)
      salp1s = salp1(s); calp1s = calp1(s);
      if f >= 0
        salp1s(str) = min(1, -x(str));
        calp1s(str) = -sqrt(1 - salp1s(str).^2);
      else
        calp1s(str) = max(cvmgt(0, -1, x(str) > -tol1), x(str));
        salp1s(str) = sqrt(1 - calp1s(str).^2);
      end
      salp1(s) = salp1s; calp1(s) = calp1s;
    end
  end

  calp1(salp1 <= 0) = 0; salp1(salp1 <= 0) = 1;
  [salp1, calp1] = SinCosNorm(salp1, calp1);
end

function k = Astroid(x, y)
% ASTROID  Solve the astroid equation
%
%   K = ASTROID(X, Y) solves the quartic polynomial Eq. (55)
%
%     K^4 + 2 * K^3 - (X^2 + Y^2 - 1) * K^2 - 2*Y^2 * K - Y^2 = 0
%
%   for the positive root K.  X and Y are column vectors of the same size
%   and the returned value K has the same size.

  k = zeros(length(x), 1);
  p = x.^2;
  q = y.^2;
  r = (p + q - 1) / 6;
  fl1 = ~(q == 0 & r <= 0);
  p = p(fl1);
  q = q(fl1);
  r = r(fl1);
  S = p .* q / 4;
  r2 = r.^2;
  r3 = r .* r2;
  disc = S .* (S + 2 * r3);
  u = r;
  fl2 = disc >= 0;
  T3 = S(fl2) + r3(fl2);
  T3 = T3 + (1 - 2 * (T3 < 0)) .* sqrt(disc(fl2));
  T = cbrt(T3);
  u(fl2) = u(fl2) + T + cvmgt(r2(fl2) ./ T, 0, T ~= 0);
  ang = atan2(sqrt(-disc(~fl2)), -(S(~fl2) + r3(~fl2)));
  u(~fl2) = u(~fl2) + 2 * r(~fl2) .* cos(ang / 3);
  v = sqrt(u.^2 + q);
  uv = u + v;
  fl2 = u < 0;
  uv(fl2) = q(fl2) ./ (v(fl2) - u(fl2));
  w = (uv - q) ./ (2 * v);
  k(fl1) = uv ./ (sqrt(uv + w.^2) + w);
end

function [lam12, dlam12, ...
          salp2, calp2, sig12, ssig1, csig1, ssig2, csig2, epsi, domg12] = ...
    Lambda12(sbet1, cbet1, dn1, sbet2, cbet2, dn2, salp1, calp1, f, A3x, C3x)
%LAMBDA12  Solve the hybrid problem

  tiny = sqrt(realmin);
  f1 = 1 - f;
  e2 = f * (2 - f);
  ep2 = e2 / (1 - e2);

  calp1(sbet1 == 0 & calp1 == 0) = -tiny;

  salp0 = salp1 .* cbet1;
  calp0 = hypot(calp1, salp1 .* sbet1);

  ssig1 = sbet1; somg1 = salp0 .* sbet1;
  csig1 = calp1 .* cbet1; comg1 = csig1;
  [ssig1, csig1] = SinCosNorm(ssig1, csig1);

  salp2 = cvmgt(salp0 ./ cbet2, salp1, cbet2 ~= cbet1);
  calp2 = cvmgt(sqrt((calp1 .* cbet1).^2 + ...
                     cvmgt((cbet2 - cbet1) .* (cbet1 + cbet2), ...
                           (sbet1 - sbet2) .* (sbet1 + sbet2), ...
                           cbet1 < -sbet1)) ./ cbet2, ...
                abs(calp1), cbet2 ~= cbet1 | abs(sbet2) ~= -sbet1);
  ssig2 = sbet2; somg2 = salp0 .* sbet2;
  csig2 = calp2 .* cbet2;  comg2 = csig2;
  [ssig2, csig2] = SinCosNorm(ssig2, csig2);

  sig12 = atan2(max(csig1 .* ssig2 - ssig1 .* csig2, 0), ...
                csig1 .* csig2 + ssig1 .* ssig2);

  omg12 = atan2(max(comg1 .* somg2 - somg1 .* comg2, 0), ...
                comg1 .* comg2 + somg1 .* somg2);
  k2 = calp0.^2 * ep2;
  epsi = k2 ./ (2 * (1 + sqrt(1 + k2)) + k2);
  C3a = C3f(epsi, C3x);
  B312 = SinCosSeries(true, ssig2, csig2, C3a) - ...
         SinCosSeries(true, ssig1, csig1, C3a);
  h0 = -f * A3f(epsi, A3x);
  domg12 = salp0 .* h0 .* (sig12 + B312);
  lam12 = omg12 + domg12;

  [~, dlam12] = ...
      Lengths(epsi, sig12, ...
              ssig1, csig1, dn1, ssig2, csig2, dn2, cbet1, cbet2, false);
  dlam12 = dlam12 .* f1 ./ (calp2 .* cbet2);
  z = calp2 == 0;
  dlam12(z) = - 2 * f1 .* dn1(z) ./ sbet1(z);
end

function [s12b, m12b, m0, M12, M21] = ...
      Lengths(epsi, sig12, ssig1, csig1, dn1, ssig2, csig2, dn2, ...
              cbet1, cbet2, scalp, ep2)
%LENGTHS  Compute various lengths associate with a geodesic

  if isempty(sig12)
    s12b = [];
    m12b = [];
    m0 = [];
    M12 = [];
    M21 = [];
    return
  end

  C1a = C1f(epsi);
  C2a = C2f(epsi);
  A1m1 = A1m1f(epsi);
  AB1 = (1 + A1m1) .* (SinCosSeries(true, ssig2, csig2, C1a) - ...
                       SinCosSeries(true, ssig1, csig1, C1a));
  A2m1 = A2m1f(epsi);
  AB2 = (1 + A2m1) .* (SinCosSeries(true, ssig2, csig2, C2a) - ...
                       SinCosSeries(true, ssig1, csig1, C2a));
  m0 = A1m1 - A2m1;
  J12 = m0 .* sig12 + (AB1 - AB2);
  m12b = dn2 .* (csig1 .* ssig2) - dn1 .* (ssig1 .* csig2) - ...
         csig1 .* csig2 .* J12;
  s12b = (1 + A1m1) .* sig12 + AB1;
  if scalp
    csig12 = csig1 .* csig2 + ssig1 .* ssig2;
    t = ep2 * (cbet1 - cbet2) .* (cbet1 + cbet2) ./ (dn1 + dn2);
    M12 = csig12 + (t .* ssig2 - csig2 .* J12) .* ssig1 ./ dn1;
    M21 = csig12 - (t .* ssig1 - csig1 .* J12) .* ssig2 ./ dn2;
  else
    M12 = sig12 + NaN; M21 = M12;
  end
end
