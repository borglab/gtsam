function y = eatanhe(x, e2)
%EATANHE   e*atanh(e*x)
%
%   EATANHE(x, e2) returns e*atanh(e*x) where e = sqrt(e2)
%   e2 is a scalar; x can be any shape.

  e = sqrt(abs(e2));
  if (e2 >= 0)
    y = e * atanh(e * x);
  else
    y = -e * atan(e * x);
  end
end
