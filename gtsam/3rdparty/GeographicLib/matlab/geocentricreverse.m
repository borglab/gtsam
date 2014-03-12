function geocentricreverse(~, ~, ~)
%geocentricreverse  Convert geocentric coordinates to geographic
%
%   [geodetic, rot] = geocentricreverse(geocentric)
%   [geodetic, rot] = geocentricreverse(geocentric, a, f)
%
%   geocentric is an M x 3 matrix of geocentric coordinates
%       x = geocentric(:,1) in meters
%       y = geocentric(:,2) in meters
%       z = geocentric(:,3) in meters
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
%   a = major radius (meters)
%   f = flattening (0 means a sphere)
%   If a and f are omitted, the WGS84 values are used.
%
% This is an interface to the GeographicLib C++ routine
%     Geocentric::Reverse
% See the documentation on this function for more information:
% http://geographiclib.sf.net/html/classGeographicLib_1_1Geocentric.html
  error('Error: executing .m file instead of compiled routine');
end
% geocentricreverse.m
% Matlab .m file for geocentric to geographic conversions
%
% Copyright (c) Charles Karney (2011) <charles@karney.com> and licensed
% under the MIT/X11 License.  For more information, see
% http://geographiclib.sourceforge.net/
