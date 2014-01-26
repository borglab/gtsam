function d = AngDiff(x, y)
%ANGDIFF  Compute angle difference accurately
%
%   D = ANGDIFF(X, Y) computes Y - X, reduces the result to (-180,180] and
%   rounds the result.  X and Y must be in [-180,180].  X and Y can be any
%   compatible shapes.

  [d, t] = sumx(-x, y);
  c = (d - 180) + t > 0;
  d(c) = (d(c) - 360) + t(c);
  c = (d + 180) + t <= 0;
  d(c) = (d(c) + 360) + t(c);
end
