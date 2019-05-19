function y = LatFix(x)
%LATFIX  Check that latitiude is in [-90, 90]
%
%   y = LATFIX(x) returns x is it is in the range [-90, 90]; otherwise it
%   returns NaN.  x can be any shape.

  y = x;
  y(abs(x) > 90) = nan;
end
