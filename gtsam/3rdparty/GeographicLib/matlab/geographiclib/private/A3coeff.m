function A3x = A3coeff(n)
%A3COEFF  Evaluate coefficients for A_3
%
%   A3x = A3COEFF(n) evaluates the coefficients of epsilon^l in Eq. (24).
%   n is a scalar.  A3x is a 1 x 6 array.

  persistent coeff nA3
  if isempty(coeff)
    nA3 = 6;
    coeff = [ ...
        -3, 128, ...
        -2, -3, 64, ...
        -1, -3, -1, 16, ...
        3, -1, -2, 8, ...
        1, -1, 2, ...
        1, 1, ...
            ];
  end
  A3x = zeros(1, nA3);
  o = 1;
  k = 1;
  for j = nA3 - 1 : -1 : 0
    m = min(nA3 - j - 1, j);
    A3x(k) = polyval(coeff(o : o + m), n) / coeff(o + m + 1);
    k = k + 1;
    o = o + m + 2;
  end
end
