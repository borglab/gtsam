function e = flat2ecc(f)
%FLAT2ECC   Convert flattening to eccentricity
%
%   e = FLAT2ECC(f)
%
%   returns the eccentricity of an ellipsoid given the flattening.
%
%   See also ECC2FLAT.

  e = sqrt(f .* (2 - f));
end
