function mgrs = mgrs_fwd(x, y, zone, isnorth, prec)
%MGRS_FWD  Convert UTM/UPS coordinates to MGRS
%
%   mgrs = MGRS_FWD(x, y, zone, isnorth)
%   mgrs = MGRS_FWD(x, y, zone, isnorth, prec)
%
%   converts from the UTM/UPS system to MGRS.  (x,y) are the easting and
%   northing (in meters); zone is the UTM zone, in [1,60], or 0 for UPS;
%   isnorth is 1 (0) for the northern (southern) hemisphere.  prec in
%   [-1,11] gives the precision of the grid reference; the default is 5
%   giving 1 m precision.  For example, prec = 2 corresponding to 1 km
%   precision, returns a string such as 38SMB4488.  A value of -1 means
%   that only the grid zone, e.g., 38S, is returned.  The maximum allowed
%   value of prec is 11 (denoting 1 um precision).  prec < -1 is treated a
%   NaN, while prec > 11 is treated the same as prec = 11.  The MGRS
%   references are returned in a cell array of strings.  x, y, zone,
%   isnorth, prec can be scalars or arrays of the same size.  Values that
%   can't be converted to MGRS return the "invalid" string, INV.  The
%   inverse operation is performed by mgrs_inv.
%
%   The allowed values of (x,y) are
%        UTM: x in [100 km, 900 km]
%             y in [0 km, 9500 km] for northern hemisphere
%             y in [1000 km, 10000 km] for southern hemisphere
%        UPS: x and y in [1300 km, 2700 km] for northern hemisphere
%             x and y in [800 km, 3200 km] for southern hemisphere
%
%   The ranges are 100 km more restrictive than for utmups_fwd and
%   utmups_inv.
%
%   See also MGRS_INV, UTMUPS_FWD.

% Copyright (c) Charles Karney (2015) <charles@karney.com>.

  narginchk(4, 5)
  if nargin < 5, prec = 5; end
  zone = floor(zone);
  prec = min(11, max(-2, floor(prec))); % this converts NaNs to -2.
  try
    s = size(x + y + zone + isnorth + prec);
  catch
    error('x, y, zone, isnorth, prec have incompatible sizes')
  end
  num = prod(s);
  if num == 0, mgrs = cell(0); return, end
  Z = zeros(num, 1);
  x = x(:) + Z; y = y(:) + Z; zone = zone(:) + Z;
  isnorth = isnorth(:) + Z; prec = prec(:) + Z;
  mgrs = repmat('INV', num, 1);
  if ~any(prec >= -1), mgrs = reshape(cellstr(mgrs), s); return, end
  maxprec = max(prec);
  mgrs = [mgrs, repmat(' ', num, 2 + 2*maxprec)];
  minprec = min(prec(prec >= -1));
  for p = minprec:maxprec
    in = prec == p;
    if ~any(in)
      continue
    end
    t = mgrs_fwd_p(x(in), y(in), zone(in), isnorth(in), p);
    mgrs(in,1:(5 + 2*p)) = t;
  end
  mgrs = reshape(cellstr(mgrs), s);
end

function mgrs = mgrs_fwd_p(x, y, zone, northp, prec)
  num = size(x, 1);
  delta = 10e-9;
  mgrs = repmat('INV', num, 1);
  mgrs = [mgrs, repmat(' ', num, 2 + 2*prec)];
  utm = zone >= 1 & zone <= 60;
  y(utm & ~northp) = y(utm & ~northp) - 100e5;
  northp(utm) = 1;
  utm = utm & x >= 1e5 & x <= 9e5 & y >= -90e5 & y <= 95e5;
  x(utm & x ==  9e5) =  9e5 - delta;
  y(utm & y == 95e5) = 95e5 - delta;
  upsn = zone == 0 &  northp;
  upss = zone == 0 & ~northp;
  upsn = upsn & x >= 13e5 & x <= 27e5 & y >= 13e5 & y <= 27e5;
  x(upsn & x == 27e5) = 27e5 - delta;
  y(upsn & y == 27e5) = 27e5 - delta;
  upss = upss & x >=  8e5 & x <= 32e5 & y >=  8e5 & y <= 32e5;
  x(upss & x == 32e5) = 32e5 - delta;
  y(upss & y == 32e5) = 32e5 - delta;
  t = mgrs_fwd_utm(x(utm), y(utm), zone(utm), prec); mgrs(utm,:) = t;
  t = mgrs_fwd_upsn(x(upsn), y(upsn), prec); mgrs(upsn,1:end-2) = t;
  t = mgrs_fwd_upss(x(upss), y(upss), prec); mgrs(upss,1:end-2) = t;
end

function mgrs = mgrs_fwd_utm(x, y, zone, prec)
  persistent latband utmcols utmrow
  if isempty(utmrow)
    latband = 'CDEFGHJKLMNPQRSTUVWX';
    utmcols = ['ABCDEFGH', 'JKLMNPQR', 'STUVWXYZ'];
    utmrow = 'ABCDEFGHJKLMNPQRSTUV';
  end
  mgrs = char(zeros(length(x), 5 + 2 * prec) + ' ');
  if isempty(x), return, end
  mgrs(:,1) = '0' + floor(zone / 10);
  mgrs(:,2) = '0' + mod(zone, 10);
  ys = y / 1e5;
  latp = 0.901 * ys + ((ys > 0) * 2 - 1) * 0.135;
  late = 0.902 * ys .* (1 - 1.85e-6 * ys .* ys);
  latp(abs(ys) < 1) = 0.9 * ys(abs(ys) < 1);
  late(abs(ys) < 1) = latp(abs(ys) < 1);
  band = LatitudeBand(latp);
  bande =  LatitudeBand(late);
  c = band ~= bande;
  band(c) = LatitudeBand(utmups_inv(x(c), y(c), zone(c), 1));
  mgrs(:,3) = latband(band + 11);
  if prec < 0, return, end
  x = floor(x * 1e6); y = floor(y * 1e6);
  xh = floor(x / 1e11); yh = floor(y / 1e11);
  mgrs(:,4) = utmcols(mod(zone - 1, 3) * 8 + xh);
  mgrs(:,5) = utmrow(mod(yh + mod(zone - 1, 2) * 5, 20) + 1);
  if prec == 0, return, end
  xy = formatnum(x, xh, y, yh, prec);
  mgrs(:,5+(1:2*prec)) = xy;
end

function mgrs = mgrs_fwd_upsn(x, y, prec)
  persistent upsband upscols upsrow
  if isempty(upsrow)
    upsband = 'YZ';
    upscols = ['RSTUXYZ', 'ABCFGHJ'];
    upsrow = 'ABCDEFGHJKLMNP';
  end
  mgrs = char(zeros(length(x), 3 + 2 * prec) + ' ');
  if isempty(x), return, end
  x = floor(x * 1e6); y = floor(y * 1e6);
  xh = floor(x / 1e11); yh = floor(y / 1e11);
  eastp = xh >= 20;
  mgrs(:,1) = upsband(eastp + 1);
  if prec < 0, return, end
  mgrs(:,2) = upscols(eastp * 7 + xh - cvmgt(20, 13, eastp) + 1);
  mgrs(:,3) = upsrow(yh - 13 + 1);
  if prec == 0, return, end
  xy = formatnum(x, xh, y, yh, prec);
  mgrs(:,3+(1:2*prec)) = xy;
end

function mgrs = mgrs_fwd_upss(x, y, prec)
  persistent upsband upscols upsrow
  if isempty(upsrow)
    upsband = 'AB';
    upscols = ['JKLPQRSTUXYZ', 'ABCFGHJKLPQR'];
    upsrow = 'ABCDEFGHJKLMNPQRSTUVWXYZ';
  end
  mgrs = char(zeros(length(x), 3 + 2 * prec) + ' ');
  if isempty(x), return, end
  x = floor(x * 1e6); y = floor(y * 1e6);
  xh = floor(x / 1e11); yh = floor(y / 1e11);
  eastp = xh >= 20;
  mgrs(:,1) = upsband(eastp + 1);
  if prec < 0, return, end
  mgrs(:,2) = upscols(eastp * 12 + xh - cvmgt(20, 8, eastp) + 1);
  mgrs(:,3) = upsrow(yh - 8 + 1);
  if prec == 0, return, end
  xy = formatnum(x, xh, y, yh, prec);
  mgrs(:,3+(1:2*prec)) = xy;
end

function xy = formatnum(x, xh, y, yh, prec)
  x = x - xh * 1e11; y = y - yh * 1e11;
  d = 10 ^ (11 - prec);
  x = floor(x / d); y = floor(y / d);
  xy = [num2str(x, ['%0', int2str(prec), '.0f']), ...
        num2str(y, ['%0', int2str(prec), '.0f'])];
end

function band = LatitudeBand(lat)
  band = max(-10, min(9, floor(lat / 8)));
  band(~(abs(lat) <= 90)) = nan;
end
