function y = cbrtx(x)
%CBRTX   The real cube root
%
%   CBRTX(x) is the real cube root of x (assuming x is real).  x
%   can be any shape.

  persistent octavep
  if isempty(octavep)
    octavep = exist('OCTAVE_VERSION', 'builtin') ~= 0;
  end
  if octavep
    y = cbrt(x);
  else
    y = abs(x).^(1/3);
    y(x < 0) = -y(x < 0);
  end
end
