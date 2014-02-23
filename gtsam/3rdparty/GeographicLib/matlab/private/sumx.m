function [s, t] = sumx(u, v)
%SUM   Error free sum
%
%   [S, T] = SUMX(U, V) returns the rounded sum U + V in S and the error in
%   T, such that S + T = U + V, exactly.  U and V can be any compatible
%   shapes.

  s = u + v;
  up = s - v;
  vpp = s - up;
  up = up - u;
  vpp = vpp -  v;
  t = -(up + vpp);
end
