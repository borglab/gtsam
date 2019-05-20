function f = ecc2flat(e)
%ECC2FLAT   Convert eccentricity to flattening
%
%   f = ECC2FLAT(e)
%
%   returns the flattening of an ellipsoid given the eccentricity.
%
%   See also FLAT2ECC.

  e2 = real(e.^2);
  f = e2 ./ (1 + sqrt(1 - e2));
end
