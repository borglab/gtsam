function A2m1 = A2m1f(epsi)
%A2M1F  Evaluate A_2 - 1
%
%   A2m1 = A2M1F(epsi) evaluates A_2 - 1 using Eq. (42).  epsi and A2m1 are
%   K x 1 arrays.

  persistent coeff
  if isempty(coeff)
    coeff = [ ...
        -11, -28, -192, 0, 256, ...
            ];
  end
  eps2 = epsi.^2;
  t = polyval(coeff(1 : end - 1), eps2) / coeff(end);
  A2m1 = (t - epsi) ./ (1 + epsi);
end
