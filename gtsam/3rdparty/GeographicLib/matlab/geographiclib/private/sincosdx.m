function [sinx, cosx] = sincosdx(x)
%SINCOSDX  Compute sine and cosine with argument in degrees
%
%   [sinx, cosx] = SINCOSDX(x) compute sine and cosine of x in degrees with
%   exact argument reduction and quadrant symmetries enforced.

  persistent octavep
  if isempty(octavep)
    octavep = exist('OCTAVE_VERSION', 'builtin') ~= 0;
  end
  if ~octavep
    % MATLAB implements argument reduction and symmetries already
    sinx = sind(x); cosx = cosd(x);
  else
    r = rem(x, 360);
    % workaround rem's bad handling of -0 in octave; fixed 2015-07-22
    % http://savannah.gnu.org/bugs/?45587
    r(x == 0 & signbit(x)) = -0;
    q = floor(r / 90 + 0.5);
    r = r - 90 * q;
    q = mod(q, 4);
    r = r * (pi/180);
    sinx = sin(r); cosx = cos(r);
    t = q == 1; z = -sinx(t); sinx(t) = cosx(t); cosx(t) = z;
    t = q == 2; sinx(t) = -sinx(t); cosx(t) = -cosx(t);
    t = q == 3; z = sinx(t); sinx(t) = -cosx(t); cosx(t) = z;
  end
  sinx(x ~= 0) = 0 + sinx(x ~= 0);
  cosx(x ~= 0) = 0 + cosx(x ~= 0);
end
