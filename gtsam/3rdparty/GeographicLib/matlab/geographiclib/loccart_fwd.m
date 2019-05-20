function [x, y, z, M] = loccart_fwd(lat0, lon0, h0, lat, lon, h, ellipsoid)
%LOCCART_FWD  Convert geographic to local cartesian coordinates
%
%   [x, y, z] = LOCCART_FWD(lat0, lon0, h0, lat, lon)
%   [x, y, z] = LOCCART_FWD(lat0, lon0, h0, lat, lon, h)
%   [x, y, z, M] = LOCCART_FWD(lat0, lon0, h0, lat, lon, h, ellipsoid)
%
%   converts from geodetic coordinates, lat, lon, h to local cartesian
%   coordinates, x, y, z, centered at lat0, lon0, h0.  Latitudes and
%   longitudes are in degrees; h (default 0), h0, x, y, z are in meters.
%   lat, lon, h can be scalars or arrays of equal size.  lat0, lon0, h0
%   must be scalars.  The ellipsoid vector is of the form [a, e], where a
%   is the equatorial radius in meters, e is the eccentricity.  If
%   ellipsoid is omitted, the WGS84 ellipsoid (more precisely, the value
%   returned by defaultellipsoid) is used.  The inverse operation is given
%   by loccart_inv.
%
%   M is the 3 x 3 rotation matrix for the conversion.  Pre-multiplying a
%   unit vector in local cartesian coordinates at (lat, lon, h) by M
%   transforms the vector to local cartesian coordinates at (lat0, lon0,
%   h0).
%
%   See also LOCCART_INV, DEFAULTELLIPSOID, FLAT2ECC.

% Copyright (c) Charles Karney (2015) <charles@karney.com>.

  narginchk(5, 7)
  if nargin < 6, h = 0; end
  if nargin < 7, ellipsoid = defaultellipsoid; end
  try
    S = size(lat + lon + h);
  catch
    error('lat, lon, h have incompatible sizes')
  end
  if ~(isscalar(lat0) && isscalar(lon0) && isscalar(h0))
    error('lat0, lon0, h0 must be scalar')
  end
  if length(ellipsoid(:)) ~= 2
    error('ellipsoid must be a vector of size 2')
  end
  num = prod(S);
  Z = zeros(num, 1);
  lat = lat(:) + Z;
  lon = lon(:) + Z;
  h = h(:) + Z;
  [X0, Y0, Z0, M0] = geocent_fwd(lat0, lon0, h0, ellipsoid);
  [X , Y , Z , M ] = geocent_fwd(lat , lon , h , ellipsoid);
  r = [X-X0, Y-Y0, Z-Z0] * M0;
  x = reshape(r(:, 1), S); y = reshape(r(:, 2), S); z = reshape(r(:, 3), S);
  if nargout > 3
    for i = 1:num
      M(:,:, i) = M0' * M(:,:, i);
    end
    M = reshape(M, [3, 3, S]);
  end
end
