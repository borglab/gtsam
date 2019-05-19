function [latlong, scale] = utmupsreverse(utmups)
%UTMUPSREVERSE  Wrapper for utmups_inv
%
%   [latlong, scale] = UTMUPSREVERSE(utmups)
%
%   This is a legacy function to replace a compiled interface function of
%   the same name.  This now calls utmups_inv which is implemented as
%   native Matlab code.
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
%   See also UTMUPS_INV.

% Copyright (c) Charles Karney (2015) <charles@karney.com>.

  [lat, lon, gam, k] = ...
      utmups_inv(utmups(:,1), utmups(:,2), utmups(:,3), utmups(:,4));
  latlong = [lat, lon];
  scale = [gam, k];
end
