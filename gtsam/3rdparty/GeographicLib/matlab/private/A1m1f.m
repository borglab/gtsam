function A1m1 = A1m1f(epsi)
%A1M1F  Evaluate A_1 - 1
%
%   A1M1 = A1M1F(EPSI) evaluates A_1 - 1 using Eq. (17).  EPSI and A1M1 are
%   K x 1 arrays.

  eps2 = epsi.^2;
  t = eps2.*(eps2.*(eps2+4)+64)/256;
  A1m1 = (t + epsi) ./ (1 - epsi);
end
