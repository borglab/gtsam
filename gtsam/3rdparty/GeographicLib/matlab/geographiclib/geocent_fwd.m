function [X, Y, Z, M] = geocent_fwd(lat, lon, h, ellipsoid)
%GEOCENT_FWD  Conversion from geographic to geocentric coordinates
%
%   [X, Y, Z] = GEOCENT_FWD(lat, lon)
%   [X, Y, Z] = GEOCENT_FWD(lat, lon, h)
%   [X, Y, Z, M] = GEOCENT_FWD(lat, lon, h, ellipsoid)
%
%   converts from geographic coordinates, lat, lon, h to geocentric
%   coordinates X, Y, Z.  lat, lon, h can be scalars or arrays of equal
%   size.  lat and lon are in degrees.  h (default 0) and X, Y, Z are in
%   meters.  The ellipsoid vector is of the form [a, e], where a is the
%   equatorial radius in meters, e is the eccentricity.  If ellipsoid is
%   omitted, the WGS84 ellipsoid (more precisely, the value returned by
%   defaultellipsoid) is used.  The inverse operation is given by
%   geocent_inv.
%
%   M is the 3 x 3 rotation matrix for the conversion.  Pre-multiplying a
%   unit vector in local cartesian coordinates (east, north, up) by M
%   transforms the vector to geocentric coordinates.
%
%   See also GEOCENT_INV, DEFAULTELLIPSOID, FLAT2ECC.

% Copyright (c) Charles Karney (2015) <charles@karney.com>.

  narginchk(2, 4)
  if nargin < 3, h = 0; end
  if nargin < 4, ellipsoid = defaultellipsoid; end
  try
    z = zeros(size(lat + lon + h));
  catch
    error('lat, lon, h have incompatible sizes')
  end
  if length(ellipsoid(:)) ~= 2
    error('ellipsoid must be a vector of size 2')
  end
  lat = LatFix(lat) + z; lon = lon + z; h = h + z;

  a = ellipsoid(1);
  e2 = real(ellipsoid(2)^2);
  e2m = 1 - e2;

  [slam, clam] = sincosdx(lon);
  [sphi, cphi] = sincosdx(lat);
  n = a./sqrt(1 - e2 * sphi.^2);
  Z = (e2m * n + h) .* sphi;
  X = (n + h) .* cphi;
  Y = X .* slam;
  X = X .* clam;

  if nargout > 3
    M = GeoRotation(sphi, cphi, slam, clam);
  end
end
