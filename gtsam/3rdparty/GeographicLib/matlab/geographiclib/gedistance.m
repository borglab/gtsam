function [s12, azi1, azi2, S12] = gedistance(lat1, lon1, lat2, lon2, ellipsoid)
%GEDISTANCE  Great ellipse distance on an ellipsoid
%
%   [s12, azi1, azi2] = GEDISTANCE(lat1, lon1, lat2, lon2)
%   [s12, azi1, azi2, S12] = GEDISTANCE(lat1, lon1, lat2, lon2, ellipsoid)
%
%   solves the inverse great ellipse problem of finding of length and
%   azimuths of the great ellipse between points specified by lat1, lon1,
%   lat2, lon2.  The input latitudes and longitudes, lat1, lon1, lat2,
%   lon2, can be scalars or arrays of equal size and must be expressed in
%   degrees.  The ellipsoid vector is of the form [a, e], where a is the
%   equatorial radius in meters, e is the eccentricity.  If ellipsoid is
%   omitted, the WGS84 ellipsoid (more precisely, the value returned by
%   defaultellipsoid) is used.  The output s12 is the distance in meters
%   and azi1 and azi2 are the forward azimuths at the end points in
%   degrees.  The optional output S12 is the area between the great ellipse
%   and the equator (in meters^2).  gedoc gives an example and provides
%   additional background information.  gedoc also gives the restrictions
%   on the allowed ranges of the arguments.
%
%   When given a combination of scalar and array inputs, the scalar inputs
%   are automatically expanded to match the size of the arrays.
%
%   geoddistance solves the equivalent geodesic problem and usually this is
%   preferable to using GEDISTANCE.
%
%   See also GEDOC, GERECKON, DEFAULTELLIPSOID, FLAT2ECC, GEODDISTANCE,
%     GEODRECKON.

% Copyright (c) Charles Karney (2014-2015) <charles@karney.com>.

  narginchk(4, 5)
  if nargin < 5, ellipsoid = defaultellipsoid; end
  try
    S = size(lat1 + lon1 + lat2 + lon2);
  catch
    error('lat1, lon1, s12, azi1 have incompatible sizes')
  end
  if length(ellipsoid(:)) ~= 2
    error('ellipsoid must be a vector of size 2')
  end
  Z = zeros(S);
  lat1 = lat1 + Z; lon1 = lon1 + Z;
  lat2 = lat2 + Z; lon2 = lon2 + Z;

  tiny = sqrt(realmin);

  a = ellipsoid(1);
  e2 = real(ellipsoid(2)^2);
  f = e2 / (1 + sqrt(1 - e2));

  f1 = 1 - f;

  areap = nargout >= 4;

  lat1 = AngRound(LatFix(lat1(:)));
  lat2 = AngRound(LatFix(lat2(:)));
  lon12 = AngRound(AngDiff(lon1(:), lon2(:)));

  [sbet1, cbet1] = sincosdx(lat1);
  sbet1 = f1 * sbet1; cbet1 = max(tiny, cbet1);
  [sbet1, cbet1] = norm2(sbet1, cbet1);

  [sbet2, cbet2] = sincosdx(lat2);
  sbet2 = f1 * sbet2; cbet2 = max(tiny, cbet2);
  [sbet2, cbet2] = norm2(sbet2, cbet2);

  [slam12, clam12] = sincosdx(lon12);

  % Solve great circle
  sgam1 = cbet2 .* slam12; cgam1 = +cbet1 .* sbet2 - sbet1 .* cbet2 .* clam12;
  sgam2 = cbet1 .* slam12; cgam2 = -sbet1 .* cbet2 + cbet1 .* sbet2 .* clam12;
  ssig12 = hypot(sgam1, cgam1);
  csig12 = sbet1 .* sbet2 + cbet1 .* cbet2 .* clam12;
  [sgam1, cgam1] = norm2(sgam1, cgam1);
  [sgam2, cgam2] = norm2(sgam2, cgam2);
  % no need to normalize [ssig12, csig12]

  cgam0 = hypot(cgam1, sgam1 .* sbet1);

  ssig1 = sbet1; csig1 = cbet1 .* cgam1;
  csig1(ssig1 == 0 & csig1 == 0) = 1;
  [ssig1, csig1] = norm2(ssig1, csig1);
  ssig2 = ssig1 .* csig12 + csig1 .* ssig12;
  csig2 = csig1 .* csig12 - ssig1 .* ssig12;

  k2 = e2 * cgam0.^2;
  epsi = k2 ./ (2 * (1 + sqrt(1 - k2)) - k2);
  C1a = C1f(epsi);
  A1 = a * (1 + A1m1f(epsi)) .* (1 - epsi)./(1 + epsi);
  s12 = A1 .* (atan2(ssig12, csig12) + ...
               (SinCosSeries(true, ssig2, csig2, C1a) - ...
                SinCosSeries(true, ssig1, csig1, C1a)));
  azi1 = atan2dx(sgam1, cgam1 .* sqrt(1 - e2 * cbet1.^2));
  azi2 = atan2dx(sgam2, cgam2 .* sqrt(1 - e2 * cbet2.^2));

  s12 = reshape(s12, S); azi1 = reshape(azi1, S); azi2 = reshape(azi2, S);

  if areap
    sgam0 = sgam1 .* cbet1;
    A4 = (a^2 * e2) * cgam0 .* sgam0;

    n = f / (2 - f);
    G4x = G4coeff(n);
    G4a = C4f(epsi, G4x);
    B41 = SinCosSeries(false, ssig1, csig1, G4a);
    B42 = SinCosSeries(false, ssig2, csig2, G4a);
    S12 = A4 .* (B42 - B41);
    S12(cgam0 == 0 | sgam0 == 0) = 0;

    sgam12 = sgam2 .* cgam1 - cgam2 .* sgam1;
    cgam12 = cgam2 .* cgam1 + sgam2 .* sgam1;
    s = sgam12 == 0 & cgam12 < 0;
    sgam12(s) = tiny * cgam1(s); cgam12(s) = -1;
    gam12 = atan2(sgam12, cgam12);

    l = abs(gam12) < 1;
    dlam12 = 1 + clam12(l); dbet1 = 1 + cbet1(l); dbet2 = 1 + cbet2(l);
    gam12(l) = ...
        2 * atan2(slam12(l) .* (sbet1(l) .* dbet2 + sbet2(l) .* dbet1), ...
                  dlam12    .* (sbet1(l) .* sbet2(l) + dbet1 .* dbet2));

    if e2 ~= 0
      c2 = a^2 * (1 + (1 - e2) * eatanhe(1, e2) / e2) / 2;
    else
      c2 = a^2;
    end
    S12 = S12 + c2 * gam12;
    S12 = reshape(S12, S);
  end
end
