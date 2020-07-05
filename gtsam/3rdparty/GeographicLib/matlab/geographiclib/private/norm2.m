function [x, y] = norm2(x, y)
%NORM2  Normalize x and y
%
%   [x, y] = NORM2(x, y) normalize x and y so that x^2 + y^2 = 1.  x and y
%   can be any shape.

  r = hypot(x, y);
  x = x ./ r;
  y = y ./ r;
end
