function [lat, lon, h, M] = loccart_inv(lat0, lon0, h0, x, y, z, ellipsoid)
%LOCCART_INV  Convert local cartesian to geographic coordinates
%
%   [lat, lon, h] = LOCCART_INV(lat0, lon0, h0, x, y, z)
%   [lat, lon, h, M] = LOCCART_INV(lat0, lon0, h0, x, y, z, ellipsoid)
%
%   converts from local cartesian coordinates, x, y, z, centered at lat0,
%   lon0, h0 to geodetic coordinates, lat, lon, h.  Latitudes and
%   longitudes are in degrees; h, h0, x, y, z are in meters.  x, y, z can
%   be scalars or arrays of equal size.  lat0, lon0, h0 must be scalars.
%   The ellipsoid vector is of the form [a, e], where a is the equatorial
%   radius in meters, e is the eccentricity.  If ellipsoid is omitted, the
%   WGS84 ellipsoid (more precisely, the value returned by
%   defaultellipsoid) is used.  The forward operation is given by
%   loccart_fwd.
%
%   M is the 3 x 3 rotation matrix for the conversion.  Pre-multiplying a
%   unit vector in local cartesian coordinates at (lat0, lon0, h0) by the
%   transpose of M transforms the vector to local cartesian coordinates at
%   (lat, lon, h).
%
%   See also LOCCART_FWD, DEFAULTELLIPSOID, FLAT2ECC.

% Copyright (c) Charles Karney (2015) <charles@karney.com>.

  narginchk(6, 7)
  if nargin < 7, ellipsoid = defaultellipsoid; end
  try
    S = size(x + y + z);
  catch
    error('x, y, z have incompatible sizes')
  end
  if ~(isscalar(lat0) && isscalar(lon0) && isscalar(h0))
    error('lat0, lon0, h0 must be scalar')
  end
  if length(ellipsoid(:)) ~= 2
    error('ellipsoid must be a vector of size 2')
  end
  num = prod(S);
  Z = zeros(num, 1);
  x = x(:) + Z;
  y = y(:) + Z;
  z = z(:) + Z;
  [X0, Y0, Z0, M0] = geocent_fwd(lat0, lon0, h0, ellipsoid);
  r = [x, y, z] * M0';
  X = r(:, 1) + X0; Y = r(:, 2) + Y0; Z = r(:, 3) + Z0;
  [lat , lon , h , M] = geocent_inv(X, Y, Z, ellipsoid);
  lat = reshape(lat, S); lon = reshape(lon, S); h = reshape(h, S);
  if nargout > 3
    for i = 1:num
      M(:,:, i) = M0' * M(:,:, i);
    end
    M = reshape(M, [3, 3, S]);
  end
end
