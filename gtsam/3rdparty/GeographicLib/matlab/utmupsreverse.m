function utmupsreverse(~)
%utmupsreverse  Convert UTM/UPS coordinates to geographic
%
%   [latlong, scale] = utmupsreverse(utmups)
%
%   utmups is an M x 4 matrix
%       easting = utmups(:,1) in meters
%       northing = utmups(:,2) in meters
%       zone = utmups(:,3)
%       hemi = utmups(:,4)
%
%   zone = 0 for UPS, zone = [1,60] for UTM
%   hemi = 0 for southern hemisphere, hemi = 1 for northern hemisphere.
%
%   latlong is an M x 2 matrix
%       latitude = latlong(:,1) in degrees
%       longitude = latlong(:,2) in degrees
%   scale is an M x 2 matrix
%       gamma = scale(:,1) meridian convergence in degrees
%       k = scale(:,2) scale
%
% This is an interface to the GeographicLib C++ routine
%     UTMUPS::Reverse
% See the documentation on this function for more information:
% http://geographiclib.sf.net/html/classGeographicLib_1_1MGRS.html
  error('Error: executing .m file instead of compiled routine');
end
% utmupsreverse.m
% Matlab .m file for UTM/UPS to geographic conversions
%
% Copyright (c) Charles Karney (2010-2011) <charles@karney.com> and licensed
% under the MIT/X11 License.  For more information, see
% http://geographiclib.sourceforge.net/
