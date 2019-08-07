function y = SinCosSeries(sinp, sinx, cosx, c)
%SINSCOSERIES  Evaluate a sine or cosine series using Clenshaw summation
%
%   y = SINCOSSERIES(sinp, sinx, cosx, c) evaluate
%     y = sum(c[i] * sin( 2*i    * x), i, 1, n), if  sinp
%     y = sum(c[i] * cos((2*i-1) * x), i, 1, n), if ~sinp
%
%   where n is the size of c.  x is given via its sine and cosine in sinx
%   and cosx.  sinp is a scalar.  sinx, cosx, and y are K x 1 arrays.  c is
%   a K x N array.

  if isempty(sinx), y = sinx; return, end
  n = size(c, 2);
  ar = 2 * (cosx - sinx) .* (cosx + sinx);
  y1 = zeros(length(sinx), 1);
  if mod(n, 2)
    y0 = c(:, n);
    n = n - 1;
  else
    y0 = y1;
  end

  for k = n : -2 : 1
    y1 = ar .* y0 - y1 + c(:, k);
    y0 = ar .* y1 - y0 + c(:, k-1);
  end
  if sinp
    y = 2 * sinx .* cosx .* y0;
  else
    y = cosx .* (y0 - y1);
  end
end
