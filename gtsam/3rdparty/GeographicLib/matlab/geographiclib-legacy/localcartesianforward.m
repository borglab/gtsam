function [cartesian, rot] = localcartesianforward(origin, geodetic, a, f)
%LOCALCARTESIANFORWARD  Wrapper for loccart_fwd
%
%   [cartesian, rot] = LOCALCARTESIANFORWARD(origin, geodetic)
%   [cartesian, rot] = LOCALCARTESIANFORWARD(origin, geodetic, a, f)
%
%   This is a legacy function to replace a compiled interface function of
%   the same name.  This now calls loccart_fwd which is implemented as
%   native Matlab code.
%
%   origin is a 1 x 3 or 1 x 2 matrix
%       lat0 = origin(1,1) in degrees
%       lon0 = origin(1,2) in degrees
%       h0 = origin(1,3) in meters (default 0 m)
%   geodetic is an M x 3 or M x 2 matrix of geodetic coordinates
%       lat = geodetic(:,1) in degrees
%       lon = geodetic(:,2) in degrees
%       h = geodetic(:,3) in meters (default 0 m)
%
%   cartesian is an M x 3 matrix of local cartesian coordinates
%       x = cartesian(:,1) in meters
%       y = cartesian(:,2) in meters
%       z = cartesian(:,3) in meters
%   rot is an M x 9 matrix
%       M = rot(:,1:9) rotation matrix in row major order.  Pre-multiplying
%           a unit vector in local cartesian coordinates at (lat, lon, h)
%           by M transforms the vector to local cartesian coordinates at
%           (lat0, lon0, h0)
%
%   a = equatorial radius (meters)
%   f = flattening (0 means a sphere)
%   If a and f are omitted, the WGS84 values are used.
%
%   See also LOCCART_FWD.

% Copyright (c) Charles Karney (2015-2017) <charles@karney.com>.

  if (nargin < 3)
    ellipsoid = defaultellipsoid;
  elseif (nargin < 4)
    ellipsoid = [a, 0];
  else
    ellipsoid = [a, flat2ecc(f)];
  end
  if size(geodetic,2) < 3
    h = 0;
  else
    h = geodetic(:,3);
  end
  if length(origin(:)) == 2
    h0 = 0;
  elseif length(origin(:)) == 3
    h0 = origin(3);
  else
    error('origin is not vector of length 2 or 3')
  end
  [x, y, z, M] = loccart_fwd(origin(1), origin(2), h0, ...
                             geodetic(:,1), geodetic(:,2), h, ellipsoid);
  cartesian = [x, y, z];
  rot = reshape(permute(M, [3, 2, 1]), [], 9);
end
