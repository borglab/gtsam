function [lat2, lon2, azi2, S12] = gereckon(lat1, lon1, s12, azi1, ellipsoid)
%GERECKON  Point along great ellipse at given azimuth and range
%
%   [lat2, lon2, azi2] = GERECKON(lat1, lon1, s12, azi1)
%   [lat2, lon2, azi2, S12] = GERECKON(lat1, lon1, s12, azi1, ellipsoid)
%
%   solves the direct great ellipse problem of finding the final point and
%   azimuth given lat1, lon1, s12, and azi1.  The input arguments lat1,
%   lon1, s12, azi1, can be scalars or arrays of equal size.  lat1, lon1,
%   azi1 are given in degrees and s12 in meters.  The ellipsoid vector is
%   of the form [a, e], where a is the equatorial radius in meters, e is
%   the eccentricity.  If ellipsoid is omitted, the WGS84 ellipsoid (more
%   precisely, the value returned by defaultellipsoid) is used.  lat2,
%   lon2, and azi2 give the position and forward azimuths at the end point
%   in degrees.  The optional output S12 is the area between the great
%   ellipse and the equator (in meters^2).  gedoc gives an example and
%   provides additional background information.  gedoc also gives the
%   restrictions on the allowed ranges of the arguments.
%
%   When given a combination of scalar and array inputs, GERECKON behaves
%   as though the inputs were expanded to match the size of the arrays.
%   However, in the particular case where lat1 and azi1 are the same for
%   all the input points, they should be specified as scalars since this
%   will considerably speed up the calculations.  (In particular a series
%   of points along a single geodesic is efficiently computed by specifying
%   an array for s12 only.)
%
%   geodreckon solves the equivalent geodesic problem and usually this is
%   preferable to using GERECKON.
%
%   See also GEDOC, GEDISTANCE, DEFAULTELLIPSOID, FLAT2ECC, GEODDISTANCE,
%     GEODRECKON.

% Copyright (c) Charles Karney (2014-2015) <charles@karney.com>.

  narginchk(4, 5)
  if nargin < 5, ellipsoid = defaultellipsoid; end
  try
    S = size(lat1 + lon1 + s12 + azi1);
  catch
    error('lat1, lon1, s12, azi1 have incompatible sizes')
  end
  if length(ellipsoid) ~= 2
    error('ellipsoid must be a vector of size 2')
  end

  tiny = sqrt(realmin);
  Z = zeros(prod(S),1);

  a = ellipsoid(1);
  e2 = real(ellipsoid(2)^2);
  f = e2 / (1 + sqrt(1 - e2));
  f1 = 1 - f;

  areap = nargout >= 4;

  lat1 = AngRound(LatFix(lat1(:)));
  lon1 = lon1(:);
  azi1 = AngRound(azi1(:));
  s12 = s12(:);

  [sgam1, cgam1] = sincosdx(azi1);
  [sbet1, cbet1] = sincosdx(lat1);
  sbet1 = f1 * sbet1; cbet1 = max(tiny, cbet1);
  [sbet1, cbet1] = norm2(sbet1, cbet1);
  [sgam1, cgam1] = norm2(sgam1 .* sqrt(1 - e2 * cbet1.^2), cgam1);
  sgam0 = sgam1 .* cbet1; cgam0 = hypot(cgam1, sgam1 .* sbet1);
  ssig1 = sbet1; slam1 = sgam0 .* sbet1;
  csig1 = cbet1 .* cgam1; csig1(sbet1 == 0 & cgam1 == 0) = 1; clam1 = csig1;
  [ssig1, csig1] = norm2(ssig1, csig1);

  k2 = e2 * cgam0.^2;
  epsi = k2 ./ (2 * (1 + sqrt(1 - k2)) - k2);
  A1 = a * (1 + A1m1f(epsi)) .* (1 - epsi)./(1 + epsi);
  C1a = C1f(epsi);
  B11 = SinCosSeries(true, ssig1, csig1, C1a);
  s = sin(B11); c = cos(B11);
  stau1 = ssig1 .* c + csig1 .* s; ctau1 = csig1 .* c - ssig1 .* s;

  C1pa = C1pf(epsi);
  tau12 = s12 ./ A1;
  s = sin(tau12); c = cos(tau12);
  B12 = - SinCosSeries(true, stau1 .* c + ctau1 .* s, ...
                       ctau1 .* c - stau1 .* s, C1pa);
  sig12 = tau12 - (B12 - B11);
  ssig12 = sin(sig12); csig12 = cos(sig12);
  if abs(f) > 0.01
    ssig2 = ssig1 .* csig12 + csig1 .* ssig12;
    csig2 = csig1 .* csig12 - ssig1 .* ssig12;
    B12 =  SinCosSeries(true, ssig2, csig2, C1a);
    serr = A1 .* (sig12 + (B12 - B11)) - s12;
    sig12 = sig12 - serr ./ (a * sqrt(1 - k2 + k2 .* ssig2.^2));
    ssig12 = sin(sig12); csig12 = cos(sig12);
  end

  ssig2 = ssig1 .* csig12 + csig1 .* ssig12;
  csig2 = csig1 .* csig12 - ssig1 .* ssig12;
  sbet2 = cgam0 .* ssig2;
  cbet2 = hypot(sgam0, cgam0 .* csig2);
  cbet2(cbet2 == 0) = tiny;
  slam2 = sgam0 .* ssig2; clam2 = csig2;
  sgam2 = sgam0; cgam2 = cgam0 .* csig2;
  lon12 = atan2dx(slam2 .* clam1 - clam2 .* slam1, ...
                  clam2 .* clam1 + slam2 .* slam1);
  lon2 = AngNormalize(AngNormalize(lon1) + lon12);
  lat2 = atan2dx(sbet2, f1 * cbet2);
  azi2 = atan2dx(sgam2, cgam2 .* sqrt(1 - e2 * cbet2.^2));

  lat2 = reshape(lat2 + Z, S);
  lon2 = reshape(lon2, S);
  azi2 = reshape(azi2 + Z, S);

  if areap
    n = f / (2 - f);
    G4x = G4coeff(n);
    G4a = C4f(epsi, G4x);
    A4 = (a^2 * e2) * cgam0 .* sgam0;
    B41 = SinCosSeries(false, ssig1, csig1, G4a);
    B42 = SinCosSeries(false, ssig2, csig2, G4a);
    sgam12 = cgam0 .* sgam0 .* ...
             cvmgt(csig1 .* (1 - csig12) + ssig12 .* ssig1, ...
                   ssig12 .* (csig1 .* ssig12 ./ (1 + csig12) + ssig1), ...
                   csig12 <= 0);
    cgam12 = sgam0.^2 + cgam0.^2 .* csig1 .* csig2;
    s = cgam0 == 0 | sgam0 == 0;
    sgam12(s) = sgam2(s) .* cgam1(s) - cgam2(s) .* sgam1(s);
    cgam12(s) = cgam2(s) .* cgam1(s) + sgam2(s) .* sgam1(s);
    s = s & sgam12 == 0 & cgam12 < 0;
    sgam12(s) = tiny * cgam1(s); cgam12(s) = -1;
    if e2 ~= 0
      c2 = a^2 * (1 + (1 - e2) * eatanhe(1, e2) / e2) / 2;
    else
      c2 = a^2;
    end
    S12 = c2 * atan2(sgam12, cgam12) + A4 .* (B42 - B41);
    S12 = reshape(S12 + Z, S);
  end

end
