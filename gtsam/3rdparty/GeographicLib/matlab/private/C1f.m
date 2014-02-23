function C1 = C1f(epsi)
%C1F  Evaluate C_{1,k}
%
%   C1 = C1F(EPSI) evaluates C_{1,l} using Eq. (18).  EPSI is a K x 1
%   array and C1 is a K x 6 array.

  nC1 = 6;
  C1 = zeros(length(epsi), nC1);
  eps2 = epsi.^2;
  d = epsi;
  C1(:,1) = d.*((6-eps2).*eps2-16)/32;
  d = d.*epsi;
  C1(:,2) = d.*((64-9*eps2).*eps2-128)/2048;
  d = d.*epsi;
  C1(:,3) = d.*(9*eps2-16)/768;
  d = d.*epsi;
  C1(:,4) = d.*(3*eps2-5)/512;
  d = d.*epsi;
  C1(:,5) = -7*d/1280;
  d = d.*epsi;
  C1(:,6) = -7*d/2048;
end
