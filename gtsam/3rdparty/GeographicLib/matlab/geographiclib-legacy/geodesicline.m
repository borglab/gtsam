function [latlong, aux] = geodesicline(lat1, lon1, azi1, distances, a, f)
%GEODESICLINE  Another wrapper for geodreckon
%
%   [latlong, aux] = GEODESICLINE(lat1, lon1, azi1, distances)
%   [latlong, aux] = GEODESICLINE(lat1, lon1, azi1, distances, a, f)
%
%   This is a legacy function to replace a compiled interface function of
%   the same name.  This now calls geodreckon which is implemented as
%   native Matlab code.
%
%   lat1 is the latitude of point 1 (scalar) in degrees
%   lon1 is the longitude of point 1 (scalar) in degrees
%   azi1 is the azimuth at point 1 (scalar) in degrees
%   distances is an M x 1 vector of distances to point 2 in meters
%
%   latlong is an M x 3 matrix
%       latitude of point 2 = geodesic(:,1) in degrees
%       longitude of point 2 = geodesic(:,2) in degrees
%       azimuth at point 2 = geodesic(:,3) in degrees
%   aux is an M x 5 matrix
%       spherical arc length = aux(:,1) in degrees
%       reduced length = aux(:,2) in meters
%       geodesic scale 1 to 2 = aux(:,3)
%       geodesic scale 2 to 1 = aux(:,4)
%       area under geodesic = aux(:,5) in meters^2
%
%   a = equatorial radius (meters)
%   f = flattening (0 means a sphere)
%   If a and f are omitted, the WGS84 values are used.
%
%   The result is the same as produced by
%       geodesicdirect([repmat([lat1, lon1, azi1],size(distances)), ...
%                       distances], a, f)
%
%   See also GEODRECKON.

% Copyright (c) Charles Karney (2015-2017) <charles@karney.com>.

  if (nargin < 5)
    ellipsoid = defaultellipsoid;
  elseif (nargin < 6)
    ellipsoid = [a, 0];
  else
    ellipsoid = [a, flat2ecc(f)];
  end
  [lat2, lon2, azi2, S12, m12, M12, M21, a12] = ...
      geodreckon(lat1, lon1, distances, azi1, ellipsoid);
  latlong = [lat2, lon2, azi2];
  aux = [a12, m12, M12, M21, S12];
end
