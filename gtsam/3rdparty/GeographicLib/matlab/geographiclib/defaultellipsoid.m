function ellipsoid = defaultellipsoid
%DEFAULTELLIPSOID  Return the WGS84 ellipsoid
%
%   ellipsoid = DEFAULTELLIPSOID
%
%   returns a vector of the equatorial radius and eccentricity for the
%   WGS84 ellipsoid.  use ecc2flat and flat2ecc to convert between
%   the eccentricity and the flattening.
%
%   See also ECC2FLAT, FLAT2ECC.

  persistent ell
  if isempty(ell)
    a = 6378137;
    f = 1/298.257223563;
    e = flat2ecc(f);
    ell = [a, e];
  end
  ellipsoid = ell;
end
