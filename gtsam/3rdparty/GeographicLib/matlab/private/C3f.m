function C3 = C3f(epsi, C3x)
%C3F  Evaluate C_3
%
%   C3 = C3F(EPSI, C3X) evaluates C_{3,l} using Eq. (25) and the
%   coefficient vector C3X.  EPSI is a K x 1 array.  C3X is a 1 x 15 array.
%   C3 is a K x 5 array.

  nC3 = 6;
  nC3x = size(C3x, 2);
  j = nC3x;
  C3 = zeros(length(epsi), nC3 - 1);
  for k = nC3 - 1 : -1 : 1
    t = C3(:, k);
    for i = nC3 - k : -1 : 1
      t = epsi .* t + C3x(j);
      j = j - 1;
    end
    C3(:, k) = t;
  end
  mult = ones(length(epsi), 1);
  for k = 1 : nC3 - 1
    mult = mult .* epsi;
    C3(:, k) = C3(:, k) .* mult;
  end
end
