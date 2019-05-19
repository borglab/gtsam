function N = geoid_height(lat, lon, geoidname, geoiddir)
%GEOID_HEIGHT  Compute the height of the geoid above the ellipsoid
%
%   N = GEOID_HEIGHT(lat, lon)
%   N = GEOID_HEIGHT(lat, lon, geoidname)
%   N = GEOID_HEIGHT(lat, lon, geoidname, geoiddir)
%       GEOID_HEIGHT([])
%   N = GEOID_HEIGHT(lat, lon, geoid)
%
%   computes the height, N, of the geoid above the WGS84 ellipsoid.  lat
%   and lon are the latitude and longitude in degrees; these can be scalars
%   or arrays of the same size.  N is in meters.
%
%   The height of the geoid above the ellipsoid, N, is sometimes called the
%   geoid undulation.  It can be used to convert a height above the
%   ellipsoid, h, to the corresponding height above the geoid (the
%   orthometric height, roughly the height above mean sea level), H, using
%   the relations
%
%       h = N + H;   H = -N + h.
%
%   The possible geoids are
%
%       egm84-30  egm84-15
%       egm96-15  egm96-5
%       egm2008-5 egm2008-2_5 egm2008-1
%
%   The first part of the name is the geoid model.  The second part gives
%   the resolution of the gridded data (in arc-seconds).  Thus egm2008-2_5
%   is the egm2008 geoid model at a resolution of 2.5".
%
%   By default the egm96-5 geoid is used.  This can be overridden by
%   specifying geoidname.  The geoiddir argument overrides the default
%   directory for the model.  See the documentation on geoid_load for how
%   these arguments are interpreted.
%
%   When geoid_height is invoked with a particular geoidname, the geoid
%   data is loaded from disk and cached.  A subsequent invocation of
%   geoid_height with the same geoidname uses the cached data.  Use
%   GEOID_HEIGHT([]) to clear this cached data.
%
%   Alternatively, you can load a geoid with geoid_load and use the
%   returned structure as the third argument.  This use does not change the
%   cached data.
%
%   In order to use this routine with Octave, Octave needs to have been
%   compiled with a version of GraphicsMagick which supports 16-bit images.
%   Also, the first time you uses this routine, you may receive a warning
%   message "your version of GraphicsMagick limits images to 16 bits per
%   pixel"; this can safely be ignored.
%
%   Information on downloading and installing the data for the supported
%   geoid models is available at
%
%     https://geographiclib.sourceforge.io/html/geoid.html#geoidinst
%
%   GEOID_HEIGHT uses cubic interpolation on gridded data that has been
%   quantized at a resolution of 3mm.
%
%   See also GEOID_LOAD.

% Copyright (c) Charles Karney (2015) <charles@karney.com>.

  persistent saved_geoid
  if nargin == 1 && isempty(lat)
    saved_geoid = [];
    return
  end
  narginchk(2, 4)
  if nargin == 3 && isstruct(geoidname)
    N = geoid_height_int(lat, lon, geoidname, true);
  else
    if nargin < 3
      geoidname = '';
    end
    if nargin < 4
      geoiddir = '';
    end
    geoidfile = geoid_file(geoidname, geoiddir);
    if ~(isstruct(saved_geoid) && strcmp(saved_geoid.file, geoidfile))
      saved_geoid = geoid_load_file(geoidfile);
    end
    N = geoid_height_int(lat, lon, saved_geoid, true);
  end
end

function N = geoid_height_int(lat, lon, geoid, cubic)
  persistent c0 c3 c0n c3n c0s c3s
  if isempty(c3s)
    c0 = 240;
    c3 = [ 9, -18, -88,    0,  96,   90,   0,   0, -60, -20;...
          -9,  18,   8,    0, -96,   30,   0,   0,  60, -20;...
           9, -88, -18,   90,  96,    0, -20, -60,   0,   0;...
         186, -42, -42, -150, -96, -150,  60,  60,  60,  60;...
          54, 162, -78,   30, -24,  -90, -60,  60, -60,  60;...
          -9, -32,  18,   30,  24,    0,  20, -60,   0,   0;...
          -9,   8,  18,   30, -96,    0, -20,  60,   0,   0;...
          54, -78, 162,  -90, -24,   30,  60, -60,  60, -60;...
         -54,  78,  78,   90, 144,   90, -60, -60, -60, -60;...
           9,  -8, -18,  -30, -24,    0,  20,  60,   0,   0;...
          -9,  18, -32,    0,  24,   30,   0,   0, -60,  20;...
           9, -18,  -8,    0, -24,  -30,   0,   0,  60,  20];
    c0n = 372;
    c3n = [ 0, 0, -131, 0,  138,  144, 0,   0, -102, -31;...
            0, 0,    7, 0, -138,   42, 0,   0,  102, -31;...
           62, 0,  -31, 0,    0,  -62, 0,   0,    0,  31;...
          124, 0,  -62, 0,    0, -124, 0,   0,    0,  62;...
          124, 0,  -62, 0,    0, -124, 0,   0,    0,  62;...
           62, 0,  -31, 0,    0,  -62, 0,   0,    0,  31;...
            0, 0,   45, 0, -183,   -9, 0,  93,   18,   0;...
            0, 0,  216, 0,   33,   87, 0, -93,   12, -93;...
            0, 0,  156, 0,  153,   99, 0, -93,  -12, -93;...
            0, 0,  -45, 0,   -3,    9, 0,  93,  -18,   0;...
            0, 0,  -55, 0,   48,   42, 0,   0,  -84,  31;...
            0, 0,   -7, 0,  -48,  -42, 0,   0,   84,  31];
    c0s = 372;
    c3s = [ 18,  -36, -122,   0,  120,  135, 0,   0,  -84, -31;...
           -18,   36,   -2,   0, -120,   51, 0,   0,   84, -31;...
            36, -165,  -27,  93,  147,   -9, 0, -93,   18,   0;...
           210,   45, -111, -93,  -57, -192, 0,  93,   12,  93;...
           162,  141,  -75, -93, -129, -180, 0,  93,  -12,  93;...
           -36,  -21,   27,  93,   39,    9, 0, -93,  -18,   0;...
             0,    0,   62,   0,    0,   31, 0,   0,    0, -31;...
             0,    0,  124,   0,    0,   62, 0,   0,    0, -62;...
             0,    0,  124,   0,    0,   62, 0,   0,    0, -62;...
             0,    0,   62,   0,    0,   31, 0,   0,    0, -31;...
           -18,   36,  -64,   0,   66,   51, 0,   0, -102,  31;...
            18,  -36,    2,   0,  -66,  -51, 0,   0,  102,  31];
  end
  try
    s = size(lat + lon);
  catch
    error('lat, lon have incompatible sizes')
  end
  num = prod(s); Z = zeros(num,1);
  lat = lat(:) + Z; lon = lon(:) + Z;
  h = geoid.h; w = geoid.w;
  % lat is in [0, h]
  flat = min(max((90 - lat) * (h - 1) / 180, 0), (h - 1));
  % lon is in [0, w)
  flon = mod(lon * w / 360, w);
  flon(isnan(flon)) = 0;
  ilat = min(floor(flat), h - 2);
  ilon = floor(flon);
  flat = flat - ilat; flon = flon - ilon;
  if ~cubic
    ind = imgind(ilon + [0,0,1,1], ilat + [0,1,0,1], w, h);
    hf = double(geoid.im(ind));
    N = (1 - flon) .* ((1 - flat) .* hf(:,1) + flat .* hf(:,2)) + ...
        flon       .* ((1 - flat) .* hf(:,3) + flat .* hf(:,4));
  else
    ind = imgind(repmat(ilon, 1, 12) + ...
                 repmat([ 0, 1,-1, 0, 1, 2,-1, 0, 1, 2, 0, 1], num, 1), ...
                 repmat(ilat, 1, 12) + ...
                 repmat([-1,-1, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2], num, 1), ...
                 w, h);
    hf = double(geoid.im(ind));
    hfx = hf * c3 / c0;
    hfx(ilat ==   0,:) = hf(ilat ==   0,:) * c3n / c0n;
    hfx(ilat == h-2,:) = hf(ilat == h-2,:) * c3s / c0s;
    N = sum(hfx .* [Z+1, flon, flat, flon.^2, flon.*flat, flat.^2, ...
                    flon.^3, flon.^2.*flat, flon.*flat.^2, flat.^3], ...
            2);
  end
  N = geoid.offset + geoid.scale * N;
  N(~(abs(lat) <= 90 & isfinite(lon))) = nan;
  N = reshape(N, s);
end

function ind = imgind(ix, iy, w, h)
% return 1-based 1d index to w*h array for 0-based 2d indices (ix,iy)
  c = iy <  0; iy(c) =           - iy(c); ix(c) = ix(c) + w/2;
  c = iy >= h; iy(c) = 2 * (h-1) - iy(c); ix(c) = ix(c) + w/2;
  ix = mod(ix, w);
  ind = 1 + iy + ix * h;
end
