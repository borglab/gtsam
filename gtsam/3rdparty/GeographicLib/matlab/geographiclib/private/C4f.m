function C4 = C4f(epsi, C4x)
%C4F  Evaluate C_4
%
%   C4 = C4F(epsi, C4x) evaluates C_{4,l} in the expansion for the area
%   (Eq. (65) expressed in terms of n and epsi) using the coefficient
%   vector C4x.  epsi is a K x 1 array.  C4x is a 1 x 21 array.  C4 is a
%   K x 6 array.

  nC4 = 6;
  C4 = zeros(length(epsi), nC4);
  mult = 1;
  o = 1;
  for l = 0 : nC4 - 1
    m = nC4 - l - 1;
    C4(:, l+1) = mult .* polyval(C4x(o : o + m), epsi);
    o = o + m + 1;
    mult = mult .* epsi;
  end
end
