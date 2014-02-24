function C3x = C3coeff(n)
%C3COEFF  Evaluate coefficients for C_3
%
%   C3x = C3COEFF(N) evaluates the coefficients of epsilon^l in Eq. (25).
%   N is a scalar.  C3x is a 1 x 15 array.

  nC3 = 6;
  nC3x = (nC3 * (nC3 - 1)) / 2;
  C3x = zeros(1, nC3x);
  C3x(0+1) = (1-n)/4;
  C3x(1+1) = (1-n*n)/8;
  C3x(2+1) = ((3-n)*n+3)/64;
  C3x(3+1) = (2*n+5)/128;
  C3x(4+1) = 3/128;
  C3x(5+1) = ((n-3)*n+2)/32;
  C3x(6+1) = ((-3*n-2)*n+3)/64;
  C3x(7+1) = (n+3)/128;
  C3x(8+1) = 5/256;
  C3x(9+1) = (n*(5*n-9)+5)/192;
  C3x(10+1) = (9-10*n)/384;
  C3x(11+1) = 7/512;
  C3x(12+1) = (7-14*n)/512;
  C3x(13+1) = 7/512;
  C3x(14+1) = 21/2560;
end
