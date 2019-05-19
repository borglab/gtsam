function [d, e] = AngDiff(x, y)
%ANGDIFF  Compute angle difference accurately
%
%   [d, e] = ANGDIFF(x, y) computes z = y - x, reduced to (-180,180].  d =
%   round(z) and e = z - round(z).  x and y can be any compatible shapes.

  [d, t] = sumx(AngNormalize(-x), AngNormalize(y));
  d = AngNormalize(d);
  d(d == 180 & t > 0) = -180;
  [d, e] = sumx(d, t);
end
