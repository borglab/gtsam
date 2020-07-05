function [area, perimeter] = polygonarea(latlong, a, f)
%POLYGONAREA  Wrapper for geodarea
%
%   [area, perimeter] = POLYGONAREA(latlong)
%   [area, perimeter] = POLYGONAREA(latlong, a, f)
%
%   This is a legacy function to replace a compiled interface function of
%   the same name.  This now calls geodarea which is implemented as native
%   Matlab code.
%
%   latlong is an M x 2 matrix
%       latitude of vertices = latlong(:,1) in degrees
%       longitude of vertices = latlong(:,2) in degrees
%
%   area is the area in meters^2
%   perimeter is the perimeter in meters
%
%   a = equatorial radius (meters)
%   f = flattening (0 means a sphere)
%   If a and f are omitted, the WGS84 values are used.
%
%   Only simple polygons (which do not intersect themselves) are supported.
%   There is no need to "close" the polygon.  Counter-clockwise traversal
%   counts as a positive area.  A polygon may encircle one or both poles.
%   The total area of the WGS84 ellipsoid is given by
%     8 * polygonarea([ 0 0; 0 90; 90 0 ])
%
%   See also GEODAREA.

% Copyright (c) Charles Karney (2015-2017) <charles@karney.com>.

  if (nargin < 2)
    ellipsoid = defaultellipsoid;
  elseif (nargin < 3)
    ellipsoid = [a, 0];
  else
    ellipsoid = [a, flat2ecc(f)];
  end
 [area, perimeter] = geodarea(latlong(:,1), latlong(:,2), ellipsoid);
end
