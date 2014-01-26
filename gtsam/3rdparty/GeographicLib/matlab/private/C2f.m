function C2 = C2f(epsi)
%C2F  Evaluate C_{2,k}
%
%   C2 = C2F(EPSI) evaluates C_{2,l} using Eq. (43).  EPSI is an
%   K x 1 array and C2 is a K x 6 array.

  nC2 = 6;
  C2 = zeros(length(epsi), nC2);
  eps2 = epsi.^2;
  d = epsi;
  C2(:,1) = d.*(eps2.*(eps2+2)+16)/32;
  d = d.*epsi;
  C2(:,2) = d.*(eps2.*(35*eps2+64)+384)/2048;
  d = d.*epsi;
  C2(:,3) = d.*(15*eps2+80)/768;
  d = d.*epsi;
  C2(:,4) = d.*(7*eps2+35)/512;
  d = d.*epsi;
  C2(:,5) = 63*d/1280;
  d = d.*epsi;
  C2(:,6) = 77*d/2048;
end
