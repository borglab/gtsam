function [geodesic, aux] = geodesicinverse(latlong, a, f)
%GEODESICINVERSE  Wrapper for geoddistance
%
%   [geodesic, aux] = GEODESICINVERSE(latlong)
%   [geodesic, aux] = GEODESICINVERSE(latlong, a, f)
%
%   This is a legacy function to replace a compiled interface function of
%   the same name.  This now calls geoddistance which is implemented as
%   native Matlab code.
%
%   latlong is an M x 4 matrix
%       latitude of point 1 = latlong(:,1) in degrees
%       longitude of point 1 = latlong(:,2) in degrees
%       latitude of point 2 = latlong(:,3) in degrees
%       longitude of point 2 = latlong(:,4) in degrees
%
%   geodesic is an M x 3 matrix
%       azimuth at point 1 = geodesic(:,1) in degrees
%       azimuth at point 2 = geodesic(:,2) in degrees
%       distance between points 1 and 2 = geodesic(:,3) in meters
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
%   See also GEODDISTANCE.

% Copyright (c) Charles Karney (2015-2017) <charles@karney.com>.

  if (nargin < 2)
    ellipsoid = defaultellipsoid;
  elseif (nargin < 3)
    ellipsoid = [a, 0];
  else
    ellipsoid = [a, flat2ecc(f)];
  end
  [s12, azi1, azi2, S12, m12, M12, M21, a12] = geoddistance ...
      (latlong(:,1), latlong(:,2), latlong(:,3), latlong(:,4), ellipsoid);
  geodesic = [azi1, azi2, s12];
  aux = [a12, m12, M12, M21, S12];
end
