function f = ecc2flat(e)
%ECC2FLAT   Convert the eccentricity of an ellipsoid to its flattening
%
%  F = ECC2FLAT(E) returns the flattening given the eccentricity.
%
%   See also FLAT2ECC.

  e2 = e.^2;
  f = e2 ./ (1 + sqrt(1 - e2));
end
