function C1p = C1pf(epsi)
%C1PF  Evaluate C'_{1,k}
%
%   C1p = C1PF(epsi) evaluates C'_{1,l} using Eq. (21).  epsi is a K x 1
%   array and C1 is a K x 6 array.

  persistent coeff nC1p
  if isempty(coeff)
    nC1p = 6;
    coeff = [ ...
        205, -432, 768, 1536, ...
        4005, -4736, 3840, 12288, ...
        -225, 116, 384, ...
        -7173, 2695, 7680, ...
        3467, 7680, ...
        38081, 61440, ...
            ];
  end
  C1p = zeros(length(epsi), nC1p);
  eps2 = epsi.^2;
  d = epsi;
  o = 1;
  for  l = 1 : nC1p
    m = floor((nC1p - l) / 2);
    C1p(:,l) = d .* polyval(coeff(o : o + m), eps2) / coeff(o + m + 1);
    o = o + m + 2;
    d = d .* epsi;
  end
end
