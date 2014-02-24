function A3x = A3coeff(n)
%A3COEFF  Evaluate coefficients for A_3
%
%   A3x = A3COEFF(N) evaluates the coefficients of epsilon^l in Eq. (24).  N
%   is a scalar.  A3x is a 1 x 6 array.

  nA3 = 6;
  A3x = zeros(1, nA3);
  A3x(0+1) = 1;
  A3x(1+1) = (n-1)/2;
  A3x(2+1) = (n*(3*n-1)-2)/8;
  A3x(3+1) = ((-n-3)*n-1)/16;
  A3x(4+1) = (-2*n-3)/64;
  A3x(5+1) = -3/128;
end
