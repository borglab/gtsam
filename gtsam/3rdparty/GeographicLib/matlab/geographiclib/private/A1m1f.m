function A1m1 = A1m1f(epsi)
%A1M1F  Evaluate A_1 - 1
%
%   A1m1 = A1M1F(epsi) evaluates A_1 - 1 using Eq. (17).  epsi and A1m1 are
%   K x 1 arrays.

  persistent coeff
  if isempty(coeff)
    coeff = [ ...
        1, 4, 64, 0, 256, ...
            ];
  end
  eps2 = epsi.^2;
  t = polyval(coeff(1 : end - 1), eps2) / coeff(end);
  A1m1 = (t + epsi) ./ (1 - epsi);
end
