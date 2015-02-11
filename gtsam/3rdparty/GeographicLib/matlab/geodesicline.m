function geodesicline(~, ~, ~, ~, ~, ~)
%geodesicline  Compute points along a geodesic
%
%   [latlong, aux] = geodesicline(lat1, lon1, azi1, distances)
%   [latlong, aux] = geodesicline(lat1, lon1, azi1, distances, a, f)
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
%   a = major radius (meters)
%   f = flattening (0 means a sphere)
%   If a and f are omitted, the WGS84 values are used.
%
%   The result is the same as produced by
%       geodesicdirect([repmat([lat1, lon1, azi1],size(distances)), ...
%                       distances], a, f)
%
% The algorithm used in this function is given in
%
%     C. F. F. Karney, Algorithms for geodesics,
%     J. Geodesy 87, 43-55 (2013);
%     http://dx.doi.org/10.1007/s00190-012-0578-z
%     Addenda: http://geographiclib.sf.net/geod-addenda.html
%
% This is an interface to the GeographicLib C++ routine
%     GeodesicLine::Position
% See the documentation on this function for more information:
% http://geographiclib.sf.net/html/classGeographicLib_1_1GeodesicLine.html
%
% A native MATLAB implementation is available as GEODRECKON.
%
% See also GEODRECKON.

  error('Error: executing .m file instead of compiled routine');
end
% geodesicline.m
% Matlab .m file for computing points along a geodesic
%
% Copyright (c) Charles Karney (2010-2011) <charles@karney.com> and licensed
% under the MIT/X11 License.  For more information, see
% http://geographiclib.sourceforge.net/
