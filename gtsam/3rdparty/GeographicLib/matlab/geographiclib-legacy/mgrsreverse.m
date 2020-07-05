function [utmups, prec] = mgrsreverse(mgrs)
%MGRSREVERSE  Wrapper for mgrs_inv
%
%   [utmups, prec] = MGRSREVERSE(mgrs)
%
%   This is a legacy function to replace a compiled interface function of
%   the same name.  This now calls mgrs_inv which is implemented as native
%   Matlab code.
%
%   mgrs is a M x 1 cell array of MGRS strings, e.g.,
%       mgrsreverse({ '38SMB4488'; '12TUK3393' })
%
%   utmups is an M x 4 matrix
%       easting = utmups(:,1) in meters
%       northing = utmups(:,2) in meters
%       zone = utmups(:,3)
%       hemi = utmups(:,4)
%   prec is an M x 1 matrix
%       precision = prec(:,1)
%                 = half the number of trailing digits in the MGRS string
%
%   zone = 0 for UPS, zone = [1,60] for UTM.
%   hemi = 0 for southern hemisphere, hemi = 1 for northern hemisphere
%   prec = precision, half the number of trailing digits
%
%   The position is the center of the MGRS square.  To obtain the
%   SW corner subtract 0.5 * 10^(5-prec) from the easting and northing.
%
%   See also MGRS_INV.

% Copyright (c) Charles Karney (2015) <charles@karney.com>.

  [x, y, z, h, prec] = mgrs_inv(mgrs);
  utmups = [x, y, z, h];
end
