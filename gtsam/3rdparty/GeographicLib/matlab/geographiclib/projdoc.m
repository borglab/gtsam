function projdoc
%PROJDOC  Projections for an ellipsoid
%
%   This package implements five projections:
%     * the transverse Mercator projection (tranmerc)
%     * the polar stereographic projection (polarst)
%     * the azimuthal equidistant projection (eqdazim)
%     * the Cassini-Soldner projection (cassini)
%     * the ellipsoidal gnomonic projection (gnomonic)
%
%   The package implements the forward projection (from geographic to
%   projected coordinates) and inverse projection (from projected to
%   geographic coordinates) with abbreviated function names (listed in
%   parentheses in the list above) suffixed by _fwd and _inv.  For each
%   function, metric properties of the projection are also returned.
%
%   The ellipsoidal gnomonic projection is defined by
%
%     [~,azi0,~,~,m,M] = geoddistance(lat0,lon0,lat,lon)
%     rho = m./M, x = rho.*sind(azi0), y = rho.*cosd(azi0)
%
%   Obviously this is an azimuthal projection.  It also enjoys
%   approximately the property of the spherical gnomonic projection, that
%   geodesics map to straight lines.  The projection is derived in Section
%   8 of
%
%     C. F. F. Karney, Algorithms for geodesics,
%     J. Geodesy 87, 43-55 (2013);
%     https://doi.org/10.1007/s00190-012-0578-z
%     Addenda: https://geographiclib.sourceforge.io/geod-addenda.html
%
%   The parameters of the ellipsoid are specified by the optional ellipsoid
%   argument to the routines.  This is a two-element vector of the form
%   [a,e], where a is the equatorial radius, e is the eccentricity e =
%   sqrt(a^2-b^2)/a, and b is the polar semi-axis.  Typically, a and b are
%   measured in meters and the linear and area quantities returned by the
%   routines are then in meters and meters^2.  However, other units can be
%   employed.  If ellipsoid is omitted, then the WGS84 ellipsoid (more
%   precisely, the value returned by defaultellipsoid) is assumed [6378137,
%   0.0818191908426215] corresponding to a = 6378137 meters and a
%   flattening f = (a-b)/a = 1/298.257223563.  The flattening and
%   eccentricity are related by
%
%       e = sqrt(f * (2 - f))
%       f = e^2 / (1 + sqrt(1 - e^2))
%
%   (The functions ecc2flat and flat2ecc implement these conversions.)  For
%   a sphere, set e = 0; for a prolate ellipsoid (b > a), specify e as a
%   pure imaginary number.
%
%   All angles (latitude, longitude, azimuth) are measured in degrees with
%   latitudes increasing northwards, longitudes increasing eastwards, and
%   azimuths measured clockwise from north.  For a point at a pole, the
%   azimuth is defined by keeping the longitude fixed, writing lat =
%   +/-(90-eps), and taking the limit eps -> 0+.
%
%   Restrictions on the inputs:
%     * All latitudes must lie in [-90, 90].
%     * The equatorial radius, a, must be positive.
%     * The eccentricity, e, should be satisfy abs(e) < 0.2 in order to
%       retain full accuracy (this corresponds to flattenings satisfying
%       abs(f) <= 1/50, approximately).  This condition holds for most
%       applications in geodesy.
%
%   See also TRANMERC_FWD, TRANMERC_INV, POLARST_FWD, POLARST_INV,
%     EQDAZIM_FWD, EQDAZIM_INV, CASSINI_FWD, CASSINI_INV, GNOMONIC_FWD,
%     GNOMONIC_INV, DEFAULTELLIPSOID, ECC2FLAT, FLAT2ECC.

% Copyright (c) Charles Karney (2012-2015) <charles@karney.com>.

  help projdoc
end
