function C2 = C2f(epsi)
%C2F  Evaluate C_{2,k}
%
%   C2 = C2F(epsi) evaluates C_{2,l} using Eq. (43).  epsi is a K x 1 array
%   and C2 is a K x 6 array.

  persistent coeff nC2
  if isempty(coeff)
    nC2 = 6;
    coeff = [ ...
        1, 2, 16, 32, ...
        35, 64, 384, 2048, ...
        15, 80, 768, ...
        7, 35, 512, ...
        63, 1280, ...
        77, 2048, ...
            ];
  end
  C2 = zeros(length(epsi), nC2);
  eps2 = epsi.^2;
  d = epsi;
  o = 1;
  for l = 1 : nC2
    m = floor((nC2 - l) / 2);
    C2(:, l) = d .* polyval(coeff(o : o + m), eps2) / coeff(o + m + 1);
    o = o + m + 2;
    d = d .* epsi;
  end
end
