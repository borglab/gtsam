function C3 = C3f(epsi, C3x)
%C3F  Evaluate C_3
%
%   C3 = C3F(epsi, C3x) evaluates C_{3,l} using Eq. (25) and the
%   coefficient vector C3x.  epsi is a K x 1 array.  C3x is a 1 x 15 array.
%   C3 is a K x 5 array.

  nC3 = 6;
  C3 = zeros(length(epsi), nC3 - 1);
  mult = 1;
  o = 1;
  for l = 1 : nC3 - 1
    m = nC3 - l - 1;
    mult = mult .* epsi;
    C3(:, l) = mult .* polyval(C3x(o : o + m), epsi);
    o = o + m + 1;
  end
end
