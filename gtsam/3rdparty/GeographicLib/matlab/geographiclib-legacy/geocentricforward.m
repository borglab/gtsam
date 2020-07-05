function [geocentric, rot] = geocentricforward(geodetic, a, f)
%GEOCENTRICFORWARD  Wrapper for geocent_fwd
%
%   [geocentric, rot] = GEOCENTRICFORWARD(geodetic)
%   [geocentric, rot] = GEOCENTRICFORWARD(geodetic, a, f)
%
%   This is a legacy function to replace a compiled interface function of
%   the same name.  This now calls geocent_fwd which is implemented as
%   native Matlab code.
%
%   geodetic is an M x 3 or M x 2 matrix of geodetic coordinates
%       lat = geodetic(:,1) in degrees
%       lon = geodetic(:,2) in degrees
%       h = geodetic(:,3) in meters (default 0 m)
%
%   geocentric is an M x 3 matrix of geocentric coordinates
%       X = geocentric(:,1) in meters
%       Y = geocentric(:,2) in meters
%       Z = geocentric(:,3) in meters
%   rot is an M x 9 matrix
%       M = rot(:,1:9) rotation matrix in row major order.  Pre-multiplying
%           a unit vector in local cartesian coordinates (east, north, up)
%           by M transforms the vector to geocentric coordinates.
%
%   a = equatorial radius (meters)
%   f = flattening (0 means a sphere)
%   If a and f are omitted, the WGS84 values are used.
%
%   See also GEOCENT_FWD.

% Copyright (c) Charles Karney (2015-2017) <charles@karney.com>.

  if (nargin < 2)
    ellipsoid = defaultellipsoid;
  elseif (nargin < 3)
    ellipsoid = [a, 0];
  else
    ellipsoid = [a, flat2ecc(f)];
  end
  if size(geodetic,2) < 3
    h = 0;
  else
    h = geodetic(:,3);
  end
  [X, Y, Z, M] = geocent_fwd(geodetic(:,1), geodetic(:,2), h, ellipsoid);
  geocentric = [X, Y, Z];
  rot = reshape(permute(M, [3, 2, 1]), [], 9);
end
