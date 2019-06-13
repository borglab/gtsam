function [geodetic, rot] = geocentricreverse(geocentric, a, f)
%GEOCENTRICREVERSE  Wrapper for geocent_inv
%
%   [geodetic, rot] = GEOCENTRICREVERSE(geocentric)
%   [geodetic, rot] = GEOCENTRICREVERSE(geocentric, a, f)
%
%   This is a legacy function to replace a compiled interface function of
%   the same name.  This now calls geocent_inv which is implemented as
%   native Matlab code.
%
%   geocentric is an M x 3 matrix of geocentric coordinates
%       X = geocentric(:,1) in meters
%       Y = geocentric(:,2) in meters
%       Z = geocentric(:,3) in meters
%
%   geodetic is an M x 3 matrix of geodetic coordinates
%       lat = geodetic(:,1) in degrees
%       lon = geodetic(:,2) in degrees
%       h = geodetic(:,3) in meters
%   rot is an M x 9 matrix
%       M = rot(:,1:9) rotation matrix in row major order.  Pre-multiplying
%           a unit vector in  geocentric coordinates by the transpose of M
%           transforms the vector to local cartesian coordinates
%           (east, north, up).
%
%   a = equatorial radius (meters)
%   f = flattening (0 means a sphere)
%   If a and f are omitted, the WGS84 values are used.
%
%   See also GEOCENT_INV.

% Copyright (c) Charles Karney (2015-2017) <charles@karney.com>.

  if (nargin < 2)
    ellipsoid = defaultellipsoid;
  elseif (nargin < 3)
    ellipsoid = [a, 0];
  else
    ellipsoid = [a, flat2ecc(f)];
  end
  [lat, lon, h, M] = geocent_inv(geocentric(:,1), geocentric(:,2), ...
                                 geocentric(:,3), ellipsoid);
  geodetic = [lat, lon, h];
  rot = reshape(permute(M, [3, 2, 1]), [], 9);
end
