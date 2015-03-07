function y = atanhee(x, e2)
%ATANHEE   atanh(e*x)/e
%
%   ATANHEE(X, E2) returns atanh(E*X)/E where E = SQRT(E2)
%   E2 is a scalar; X can be any shape.

  e = sqrt(abs(e2));
  if (e2 > 0)
    y = atanh(e * x) / e;
  elseif (e2 < 0)
    y = atan(e * x) / e;
  else
    y = x;
  end
end
