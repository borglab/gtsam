function [lat, lon, gam, k] = utmups_inv(x, y, zone, isnorth)
%UTMUPS_INV  Convert from UTM/UPS system
%
%   [lat, lon] = UTMUPS_INV(x, y, zone, isnorth)
%   [lat, lon, gam, k] = UTMUPS_INV(x, y, zone, isnorth)
%
%   convert to the UTM/UPS system to geographical coordinates, (lat,lon).
%   The input is (x,y) = (easting,northing), the zone which is either the
%   UTM zone or 0 for UPS , and a hemisphere selector, isnorth (0 for the
%   southern hemisphere, 1 for the northern).  x, y, zone, and isnorth can
%   be scalars or arrays of equal size.  The forward operation is performed
%   by utmups_fwd.
%
%   gam and k give metric properties of the projection at (lat,lon); gam is
%   the meridian convergence at the point and k is the scale.
%
%   lat, lon, gam are in degrees.  The projected coordinates x, y are in
%   meters.  k is dimensionless.
%
%   The argument zone has the following meanings
%        0, use UPS
%        [1,60], use the corresponding UTM zone
%       -4, an undefined zone
%
%   The allowed values of (x,y) are
%        UTM: x in [0 km, 1000 km]
%             y in [0 km, 9600 km] for northern hemisphere
%             y in [900 km, 10000 km] for southern hemisphere
%        UPS: x and y in [1200 km, 2800 km] for northern hemisphere
%             x and y in [700 km, 3300 km] for southern hemisphere
%
%   The ranges are 100 km less restrictive than for mgrs_fwd.
%
%   UTMUPS_INV checks that (x,y) lie in the limits given above.  If these
%   conditions don't hold (lat,lon), gam, k are converted to NaN.
%
%   See also UTMUPS_FWD, TRANMERC_INV, POLARST_INV, MGRS_INV.

% Copyright (c) Charles Karney (2015-2016) <charles@karney.com>.

  narginchk(4, 4)
  try
    Z = zeros(size(x + y + zone + isnorth));
  catch
    error('x, y, zone, isnorth have incompatible sizes')
  end
  x = x + Z; y = y + Z;
  zone = floor(zone) + Z; isnorth = logical(isnorth + Z);
  Z = nan(size(Z));
  lat = Z; lon = Z; gam = Z; k = Z;
  utm = zone > 0 & zone <= 60;
  [lat(utm), lon(utm), gam(utm), k(utm)] = ...
      utm_inv(zone(utm), isnorth(utm), x(utm), y(utm));
  ups = zone == 0;
  [lat(ups), lon(ups), gam(ups), k(ups)] = ...
      ups_inv(isnorth(ups), x(ups), y(ups));
end

function [lat, lon, gam, k] = utm_inv(zone, isnorth, x, y)
%UTM_INV  Inverse UTM projection

  lon0 = -183 + 6 * floor(zone); lat0 = 0;
  fe = 5e5; fn = 100e5 * (1-isnorth); k0 = 0.9996;
  x = x - fe; y = y - fn;
  bad = ~(abs(x) <= 5e5 & y >= -91e5 & y <= 96e5);
  x = x / k0; y = y / k0;
  [lat, lon, gam, k] = tranmerc_inv(lat0, lon0, x, y);
  k = k * k0;
  lat(bad) = nan; lon(bad) = nan; gam(bad) = nan; k(bad) = nan;
end

function [lat, lon, gam, k] = ups_inv(isnorth, x, y)
%UPS_INV  Inverse UPS projection

  fe = 20e5; fn = 20e5; k0 = 0.994;
  x = x - fe; y = y - fn;
  lim = (13 - 5 * isnorth) * 1e5;
  bad = ~(abs(x) <= lim & abs(y) <= lim);
  x = x / k0; y = y / k0;
  [lat, lon, gam, k] = polarst_inv(isnorth, x, y);
  k = k * k0;
  lat(bad) = nan; lon(bad) = nan; gam(bad) = nan; k(bad) = nan;
end
