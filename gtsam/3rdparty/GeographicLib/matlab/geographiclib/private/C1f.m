function C1 = C1f(epsi)
%C1F  Evaluate C_{1,k}
%
%   C1 = C1F(epsi) evaluates C_{1,l} using Eq. (18).  epsi is a K x 1
%   array and C1 is a K x 6 array.

  persistent coeff nC1
  if isempty(coeff)
    nC1 = 6;
    coeff = [ ...
        -1, 6, -16, 32, ...
        -9, 64, -128, 2048, ...
        9, -16, 768, ...
        3, -5, 512, ...
        -7, 1280, ...
        -7, 2048, ...
            ];
  end
  C1 = zeros(length(epsi), nC1);
  eps2 = epsi.^2;
  d = epsi;
  o = 1;
  for  l = 1 : nC1
    m = floor((nC1 - l) / 2);
    C1(:,l) = d .* polyval(coeff(o : o + m), eps2) / coeff(o + m + 1);
    o = o + m + 2;
    d = d .* epsi;
  end
end
