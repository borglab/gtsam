function [lat, lon, gam, k] = utm_inv(zone, northp, x, y)
%UTM_INV  Forward transverse Mercator projection
%
%   [LAT, LON] = UTM_INV(ZONE, NORTHP, X, Y)
%   [LAT, LON, GAM, K] = UTM_INV(ZONE, NORTHP, X, Y)
%
%   performs the inverse universal transverse Mercator projection of points
%   (X,Y) to (LAT,LON) using ZONE and NORTHP.  X and Y can be scalars or
%   arrays of equal size.  ZONE should be an integer in [1,60] and NORTHP
%   is a logical indicating whether the transformation should use the false
%   northing for the northern (NORTHP = true) or southern (NORTHP = false)
%   hemisphere.  The forward projection is given by UTM_FWD.
%
%   GAM and K give metric properties of the projection at (LAT,LON); GAM is
%   the meridian convergence at the point and K is the scale.
%
%   LAT, LON, GAM are in degrees.  The projected coordinates X, Y are in
%   meters.  K is dimensionless.
%
%   This implementation for the UTM projection is based on the series
%   method described in
%
%     C. F. F. Karney, Transverse Mercator with an accuracy of a few
%     nanometers, J. Geodesy 85(8), 475-485 (Aug. 2011);
%     Addenda: http://geographiclib.sf.net/tm-addenda.html
%
%   This extends the series given by Krueger (1912) to sixth order in the
%   flattening.  This is a substantially better series than that used by
%   the MATLAB mapping toolbox.  In particular the errors in the projection
%   are less than 5 nanometers withing 3900 km of the central meridian (and
%   less than 1 mm within 7600 km of the central meridian).  The mapping
%   can be continued accurately over the poles to the opposite meridian.
%
%   This routine depends on the MATLAB File Exchange package "Geodesics on
%   an ellipsoid of revolution":
%
%     http://www.mathworks.com/matlabcentral/fileexchange/39108
%
%   See also GEODPROJ, UTM_FWD, TRANMERC_INV.

% Copyright (c) Charles Karney (2012) <charles@karney.com>.
%
% This file was distributed with GeographicLib 1.29.

  if nargin < 4, error('Too few input arguments'), end
  lon0 = -183 + 6 * zone; lat0 = 0;
  fe = 500e3; fn = cvmgt(0, 10000e3, logical(northp)); k0 = 0.9996;
  x = (x - fe) / k0; y = (y - fn) / k0;
  [lat, lon, gam, k] = tranmerc_inv(lat0, lon0, x, y);
  k = k * k0;
end
