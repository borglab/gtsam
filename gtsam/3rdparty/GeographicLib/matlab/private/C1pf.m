function c = C1pf(epsi)
%C1PF  Evaluate C'_{1,k}
%
%   C1P = C1PF(EPSI) evaluates C'_{1,l} using Eq. (21).  EPSI is an
%   K x 1 array and C1 is a K x 6 array.

  nC1p = 6;
  c = zeros(length(epsi), nC1p);
  eps2 = epsi.^2;
  d = epsi;
  c(:,1) = d.*(eps2.*(205*eps2-432)+768)/1536;
  d = d.*epsi;
  c(:,2) = d.*(eps2.*(4005*eps2-4736)+3840)/12288;
  d = d.*epsi;
  c(:,3) = d.*(116-225*eps2)/384;
  d = d.*epsi;
  c(:,4) = d.*(2695-7173*eps2)/7680;
  d = d.*epsi;
  c(:,5) = 3467*d/7680;
  d = d.*epsi;
  c(:,6) = 38081*d/61440;
end
