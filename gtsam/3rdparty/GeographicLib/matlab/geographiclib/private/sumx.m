function [s, t] = sumx(u, v)
%SUM   Error free sum
%
%   [s, t] = SUMX(u, v) returns the rounded sum u + v in s and the error in
%   t, such that s + t = u + v, exactly.  u and v can be any compatible
%   shapes.

  s = u + v;
  up = s - v;
  vpp = s - up;
  up = up - u;
  vpp = vpp -  v;
  t = -(up + vpp);
end
