function A2m1 = A2m1f(epsi)
%A2M1F  Evaluate A_2 - 1
%
%   A2M1 = A2M1F(EPSI) evaluates A_2 - 1 using Eq. (42).  EPSI and A2M1 are
%   K x 1 arrays.

  eps2 = epsi.^2;
  t = eps2.*(eps2.*(25*eps2+36)+64)/256;
  A2m1 = t .* (1 - epsi) - epsi;
end
