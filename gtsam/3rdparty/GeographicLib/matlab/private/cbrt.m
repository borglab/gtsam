function y = cbrt(x)
%CBRT   The real cube root
%
%   CBRT(X) is the real cube root of X (assuming X is real).  X
%   can be any shape.

  y = abs(x).^(1/3);
  y(x < 0) = -y(x < 0);
end
