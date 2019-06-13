function z = copysignx(x, y)
%COPYSIGNX   Copy the sign
%
%   COPYSIGNX(x,y) returns the magnitude of x with the sign of y.  x and y
%   can be any compatible shapes.

  persistent octavep
  if isempty(octavep)
    octavep = exist('OCTAVE_VERSION', 'builtin') ~= 0;
  end
  if octavep
    z = abs(x) .* (1 - 2 * signbit(y));
  else
    z = abs(x + 0*y);
    l = z > -1 & (y < 0 | (y == 0 & 1./y < 0));
    z(l) = -z(l);
  end
end
