function mgrs = mgrsforward(utmups, prec)
%MGRSFORWARD  Wrapper for mgrs_fwd
%
%   mgrs = MGRSFORWARD(utmups)
%   mgrs = MGRSFORWARD(utmups, prec)
%
%   This is a legacy function to replace a compiled interface function of
%   the same name.  This now calls mgrs_fwd which is implemented as native
%   Matlab code.
%
%   utmups is an M x 4 matrix
%       easting = utmups(:,1) in meters
%       northing = utmups(:,2) in meters
%       zone = utmups(:,3)
%       hemi = utmups(:,4)
%
%   zone = 0 for UPS, zone = [1,60] for UTM
%   hemi = 0 for southern hemisphere, hemi = 1 for northern hemisphere
%   prec = half the number of trailing digits in the MGRS string
%          (default 5)
%
%   mgrs is a M x 1 cell array of MGRS strings.
%
%   See also MGRS_FWD.

% Copyright (c) Charles Karney (2015) <charles@karney.com>.

  if nargin < 2
    prec = 5;
  end
  mgrs = mgrs_fwd(utmups(:,1), utmups(:,2), utmups(:,3), utmups(:,4), prec);
end
