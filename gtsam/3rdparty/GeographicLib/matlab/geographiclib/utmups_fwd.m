function [x, y, zone, isnorth, gam, k] = utmups_fwd(lat, lon, setzone)
%UTMUPS_FWD  Convert to UTM/UPS system
%
%   [x, y, zone, isnorth] = UTMUPS_FWD(lat, lon)
%   [x, y, zone, isnorth, gam, k] = UTMUPS_FWD(lat, lon, setzone)
%
%   convert from geographical coordinates, (lat,lon), to the UTM/UPS
%   system.  The output is (x,y) = (easting,northing), zone which is either
%   the UTM zone or 0 for UPS, and a hemisphere selector, isnorth (0 for
%   the southern hemisphere, 1 for the northern).  If setzone = -1 (the
%   default), the standard choice is made between UTM and UPS and, if UTM,
%   the standard zone is picked (the Norway and Svalbard exceptions are
%   honored).  lat, lon, and setzone can be scalars or arrays of equal
%   size.  The inverse operation is performed by utmups_inv.
%
%   gam and k give metric properties of the projection at (lat,lon); gam is
%   the meridian convergence at the point and k is the scale.
%
%   lat, lon, gam are in degrees.  The projected coordinates x, y are in
%   meters.  k is dimensionless.
%
%   The optional argument, setzone, allows the UTM/UPS and zone
%   assignment to be overridden.  The choices are
%        0, use UPS
%        [1,60], use the corresponding UTM zone
%       -1, use the standard assigment (the default)
%       -2, use closest UTM zone
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
%   UTMUPS_FWD checks its arguments and requires that lat is in
%   [-90deg,90deg] and that (x,y) lie in the limits given above.  If these
%   conditions don't hold (x,y), gam, k are converted to NaN, zone to -4
%   and isnorthp to 0.
%
%   See also UTMUPS_INV, TRANMERC_FWD, POLARST_FWD, MGRS_FWD.

% Copyright (c) Charles Karney (2015-2016) <charles@karney.com>.

  narginchk(2, 3)
  if nargin < 3, setzone = -1; end
  try
    Z = zeros(size(lat + lon + setzone));
  catch
    error('lat, lon, setzone have incompatible sizes')
  end
  lat = lat + Z; lon = lon + Z;
  isnorth = lat >= 0;
  zone = StandardZone(lat, lon, setzone);
  Z = nan(size(Z));
  x = Z; y = Z; gam = Z; k = Z;
  utm = zone > 0;
  [x(utm), y(utm), gam(utm), k(utm)] = ...
      utm_fwd(zone(utm), isnorth(utm), lat(utm), lon(utm));
  ups = zone == 0;
  [x(ups), y(ups), gam(ups), k(ups)] = ...
      ups_fwd(isnorth(ups), lat(ups), lon(ups));
  zone(isnan(x)) = -4; isnorth(isnan(x)) = false;
end

function [x, y, gam, k] = utm_fwd(zone, isnorth, lat, lon)
%UTM_FWD  Forward UTM projection

  lon0 = -183 + 6 * floor(zone); lat0 = 0;
  bad = ~(abs(mod(lon - lon0 + 180, 360) - 180) <= 60);
  fe = 5e5; fn = 100e5 * (1-isnorth); k0 = 0.9996;
  [x, y, gam, k] = tranmerc_fwd(lat0, lon0, lat, lon);
  x = x * k0; y = y * k0; k = k * k0;
  bad = bad | ~(abs(x) <= 5e5 & y >= -91e5 & y <= 96e5);
  x = x + fe; y = y + fn;
  x(bad) = nan; y(bad) = nan; gam(bad) = nan; k(bad) = nan;
end

function [x, y, gam, k] = ups_fwd(isnorth, lat, lon)
%UPS_FWD  Forward UPS projection

  fe = 20e5; fn = 20e5; k0 = 0.994;
  [x, y, gam, k] = polarst_fwd(isnorth, lat, lon);
  x = x * k0; y = y * k0; k = k * k0;
  lim = (13 - 5 * isnorth) * 1e5;
  bad = ~(abs(x) <= lim & abs(y) <= lim);
  x = x + fe; y = y + fn;
  x(bad) = nan; y(bad) = nan; gam(bad) = nan; k(bad) = nan;
end

function zone = StandardZone(lat, lon, setzone)
  INVALID = -4;
  UTM = -2;
  MINZONE = 0;
  MAXZONE = 60;
  UPS = 0;
  zone = floor(setzone) + zeros(size(lat));
  zone(~(zone >= INVALID & zone <= MAXZONE)) = INVALID;
  g = zone < MINZONE & zone ~= INVALID;
  c = abs(lat) <= 90 & isfinite(lon);
  zone(g & ~c) = INVALID;
  g = g & c;
  c = zone == UTM | (lat >= -80 & lat < 84);
  u = g & c;
  ilon = mod(floor(lon(u)) + 180, 360) - 180;
  z = floor((ilon + 186) / 6);
  % Norway exception
  exception = z == 31 & floor(lat(u) / 8) == 7 & ilon >= 3;
  z(exception) = 32;
  % Svalbard exception
  exception = lat(u) >= 72 & ilon >= 0 & ilon < 42;
  z(exception) = 2 * floor((ilon(exception) + 183)/12) + 1;
  zone(u) = z;
  zone(g & ~c) = UPS;
end
