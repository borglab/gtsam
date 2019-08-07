function [lat, lon, h, M] = geocent_inv(X, Y, Z, ellipsoid)
%GEOCENT_INV  Conversion from geocentric to geographic coordinates
%
%   [lat, lon, h] = GEOCENT_INV(X, Y, Z)
%   [lat, lon, h, M] = GEOCENT_INV(X, Y, Z, ellipsoid)
%
%   converts from geocentric coordinates X, Y, Z to geographic coordinates,
%   lat, lon, h.  X, Y, Z can be scalars or arrays of equal size.  X, Y, Z,
%   and h are in meters.  lat and lon are in degrees.  The ellipsoid vector
%   is of the form [a, e], where a is the equatorial radius in meters, e is
%   the eccentricity.  If ellipsoid is omitted, the WGS84 ellipsoid (more
%   precisely, the value returned by defaultellipsoid) is used.  The
%   forward operation is given by geocent_fwd.
%
%   M is the 3 x 3 rotation matrix for the conversion.  Pre-multiplying a
%   unit vector in geocentric coordinates by the transpose of M transforms
%   the vector to local cartesian coordinates (east, north, up).
%
%   See also GEOCENT_FWD, DEFAULTELLIPSOID, FLAT2ECC.

% Copyright (c) Charles Karney (2015) <charles@karney.com>.

  narginchk(3, 4)
  if nargin < 4, ellipsoid = defaultellipsoid; end
  try
    z = zeros(size(X + Y + Z));
  catch
    error('X, Y, Z have incompatible sizes')
  end
  if length(ellipsoid(:)) ~= 2
    error('ellipsoid must be a vector of size 2')
  end
  X = X + z; Y = Y + z; Z = Z + z;

  a = ellipsoid(1);
  e2 = real(ellipsoid(2)^2);
  e2m = 1 - e2;
  e2a = abs(e2);
  e4a = e2^2;
  maxrad = 2 * a / eps;

  R = hypot(X, Y);
  slam = Y ./ R; slam(R == 0) = 0;
  clam = X ./ R; clam(R == 0) = 1;
  h = hypot(R, Z);

  if e4a == 0
    % Treat the spherical case.  Dealing with underflow in the general case
    % with e2 = 0 is difficult.  Origin maps to N pole same as with
    % ellipsoid.
    Z1 = Z;
    Z1(h == 0) = 1;
    [sphi, cphi] = norm2(Z1, R);
    h = h - a;
  else
    % Treat prolate spheroids by swapping R and Z here and by switching
    % the arguments to phi = atan2(...) at the end.
    p = (R / a).^2;
    q = e2m * (Z / a).^2;
    r = (p + q - e4a) / 6;
    if e2 < 0
      [p, q] = swap(p, q);
    end

    % Avoid possible division by zero when r = 0 by multiplying
    % equations for s and t by r^3 and r, resp.
    S = e4a * p .* q / 4; % S = r^3 * s
    r2 = r.^2;
    r3 = r .* r2;
    disc = S .* (2 * r3 + S);
    u = r;
    fl2 = disc >= 0;
    T3 = S(fl2) + r3(fl2);
    % Pick the sign on the sqrt to maximize abs(T3).  This minimizes
    % loss of precision due to cancellation.  The result is unchanged
    % because of the way the T is used in definition of u.
    % T3 = (r * t)^3
    T3 = T3 + (1 - 2 * (T3 < 0)) .* sqrt(disc(fl2));
    % N.B. cbrtx always returns the real root.  cbrtx(-8) = -2.
    T = cbrtx(T3);
    u(fl2) = u(fl2) + T + cvmgt(r2(fl2) ./ T, 0, T ~= 0);
    % T is complex, but the way u is defined the result is real.
    ang = atan2(sqrt(-disc(~fl2)), -(S(~fl2) + r3(~fl2)));
    % There are three possible cube roots.  We choose the root which
    % avoids cancellation (disc < 0 implies that r < 0).
    u(~fl2) = u(~fl2) + 2 * r(~fl2) .* cos(ang / 3);
    % guaranteed positive
    v = sqrt(u.^2 + e4a * q);
    % Avoid loss of accuracy when u < 0.  Underflow doesn't occur in
    % e4 * q / (v - u) because u ~ e^4 when q is small and u < 0.
    % u+v, guaranteed positive
    uv = u + v;
    fl2 = u < 0;
    uv(fl2) = e4a * q(fl2) ./ (v(fl2) - u(fl2));
    % Need to guard against w going negative due to roundoff in uv - q.
    w = max(0, e2a * (uv - q) ./ (2 * v));
    k = uv ./ (sqrt(uv + w.^2) + w);
    if e2 >= 0
      k1 = k; k2 = k + e2;
    else
      k1 = k - e2; k2 = k;
    end
    [sphi, cphi] = norm2(Z ./ k1, R ./ k2);
    h = (1 - e2m ./ k1) .* hypot(k1 .* R ./ k2, Z);
    % Deal with exceptional inputs
    c = e4a * q == 0 & r <= 0;
    if any(c)
      % This leads to k = 0 (oblate, equatorial plane) and k + e^2 = 0
      % (prolate, rotation axis) and the generation of 0/0 in the general
      % formulas for phi and h.  using the general formula and division by 0
      % in formula for h.  So handle this case by taking the limits:
      % f > 0: z -> 0, k      ->   e2 * sqrt(q)/sqrt(e4 - p)
      % f < 0: R -> 0, k + e2 -> - e2 * sqrt(q)/sqrt(e4 - p)
      zz = e4a - p(c); xx = p(c);
      if e2 < 0
        [zz, xx] = swap(zz, xx);
      end
      zz = sqrt(zz / e2m);
      xx = sqrt(xx);
      H = hypot(zz, xx); sphi(c) = zz ./ H; cphi(c) = xx ./ H;
      sphi(c & Z < 0) = - sphi(c & Z < 0);
      h(c) = - a *  H / e2a;
      if e2 >= 0
        h(c) = e2m * h(c);
      end
    end
  end
  far = h > maxrad;
  if any(far)
    % We really far away (> 12 million light years); treat the earth as a
    % point and h, above, is an acceptable approximation to the height.
    % This avoids overflow, e.g., in the computation of disc below.  It's
    % possible that h has overflowed to inf; but that's OK.
    %
    % Treat the case X, Y finite, but R overflows to +inf by scaling by 2.
    R(far) = hypot(X(far)/2, Y(far)/2);
    slam(far) = Y(far) ./ R(far); slam(far & R == 0) = 0;
    clam(far) = X(far) ./ R(far); clam(far & R == 0) = 1;
    H = hypot(Z(far)/2, R(far));
    sphi(far) = Z(far)/2 ./ H;
    cphi(far) = R(far) ./ H;
  end
  lat = atan2dx(sphi, cphi);
  lon = atan2dx(slam, clam);
  if nargout > 3
    M = GeoRotation(sphi, cphi, slam, clam);
  end
end
