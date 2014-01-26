function geodproj
%GEODPROJ  Geodesic projections for an ellipsoid
%
%   This package implements four projections based on geodesics:
%     * the azimuthal equidistant projection (eqdazim)
%     * the Cassini-Soldner projection (cassini)
%     * the transverse Mercator projection (tranmerc)
%     * the ellipsoidal gnomonic projection (gnomonic)
%
%   The package implements the forward projection (from geographic to
%   projected coordinates) and inverse projection (from projected to
%   geographic coordinates) with abbreviated function names (listed in
%   parentheses in the list above) suffixed by _FWD and _INV.  In addition,
%   implementations of the UTM projection are provided with UTM_FWD and
%   UTM_INV.  For each function, metric properties of the projection are
%   also returned.
%
%   This package requires the availability of MATLAB File Exchange package
%   "Geodesics on an ellipsoid of revolution" for performing the necessary
%   geodesic computations:
%
%     http://www.mathworks.com/matlabcentral/fileexchange/39108
%
%   The azimuthal equidistant inverse projection is a geodesic projection
%   defined by
%
%     [S,AZI0] = GEODDISTANCE(LAT0,LON0,LAT,LON)
%     X = S.*SIND(AZI0), Y = S.*COSD(AZI0)
%
%   Thus all distances and azimuths relative to the center point are
%   correct.
%
%   The Cassini-Solder projection is a rectangular geodesic projection
%   whose inverse is specified by
%
%     [LAT1,LON1,AZI1] = GEODRECKON(LAT0,LON0,Y,0)
%     [LAT,LON] = GEODRECKON(LAT1,LON1,X,AZI1+90)
%
%   Cassini-Soldner is a transverse cylindrical projection where the
%   meridian LON0 maps to a straight line with constant scale and distances
%   perpendicular to the central meridian are true.  Cassini-Soldner was
%   widely used for large scale maps until about 1930 (when it was
%   supplanted by various conformal projections, in particular, by the
%   transverse Mercator projection).
%
%   The transverse Mercator projection is also a cylindrical projection
%   which maps the central meridian to a straight line at constant scale.
%   However the behavior either side of the central meridian is determined
%   by the condition of conformality.  The implementation used in this
%   package is based on the series method described in
%
%     C. F. F. Karney, Transverse Mercator with an accuracy of a few
%     nanometers, J. Geodesy 85(8), 475-485 (Aug. 2011);
%     Addenda: http://geographiclib.sf.net/tm-addenda.html
%
%   The ellipsoidal gnomonic projection is defined by
%
%     [~,AZI0,~,~,m,M] = GEODDISTANCE(LAT0,LON0,LAT,LON)
%     RHO = m./M, X = RHO.*SIND(AZI0), Y = RHO.*COSD(AZI0)
%
%   Obviously this is an azimuthal projection.  It also enjoys approximately
%   the property of the spherical gnomonic projection, that geodesics map
%   to straight lines.  The projection is derived in Section 8 of
%
%     C. F. F. Karney, Algorithms for geodesics,
%     J. Geodesy 87, 43-55 (2013);
%     http://dx.doi.org/10.1007/s00190-012-0578-z
%     Addenda: http://geographiclib.sf.net/geod-addenda.html
%
%   The parameters of the ellipsoid are specified by the optional ELLIPSOID
%   argument to the routines.  This is a two-element vector of the form
%   [a,e], where a is the equatorial radius, e is the eccentricity e =
%   sqrt(a^2-b^2)/a, and b is the polar semi-axis.  Typically, a and b are
%   measured in meters and the linear and area quantities returned by the
%   routines are then in meters and meters^2.  However, other units can be
%   employed.  If ELLIPSOID is omitted, then the WGS84 ellipsoid (more
%   precisely, the value returned by DEFAULTELLIPSOID) is assumed [6378137,
%   0.0818191908426215] corresponding to a = 6378137 meters and a
%   flattening f = (a-b)/a = 1/298.257223563.  The flattening and
%   eccentricity are related by
%
%       e = sqrt(f * (2 - f))
%       f = e^2 / (1 + sqrt(1 - e^2))
%
%   (The functions ECC2FLAT and FLAT2ECC implement these conversions.)  For
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
%     * All longitudes and azimuths must lie in [-540, 540).  On output,
%       these quantities lie in [-180, 180).
%     * The equatorial radius, a, must be positive.
%     * The eccentricity, e, should be satisfy abs(e) < 0.2 in order to
%       retain full accuracy (this corresponds to flattenings satisfying
%       abs(f) <= 1/50, approximately).  This condition holds for most
%       applications in geodesy.
%
%   These routines are fully vectorized so that their speed is competitive
%   with the compiled C++ code.  These MATLAB versions are transcriptions
%   of several C++ classes provided by GeographicLib which is available at
%
%     http://geographiclib.sf.net
%
%   The routines duplicate some of the functionality of the EQDAZIM,
%   CASSINISTD, TRANMERC, and GNOMONIC projections in the the MATLAB
%   mapping toolbox.  The major improvements offered by this package are
%
%     * The azimuthal equidistant and gnomonic projections are defined
%       for the ellipsoid (instead of just for the sphere).
%     * The Cassini-Soldner projection is essentially exact (instead of
%       being defined in terms of an approximate series).
%     * The transverse Mercator projection uses a much more accurate
%       series which greatly extends its domain of applicability.
%
%   The primary importance of the two azimuthal projections is that they
%   offer a convenient way of solving various geometrical problems on the
%   ellipsoid.  In particular the azimuthal equidistant projection allows
%   problems associated with determining maritime boundaries to be solved
%   easily.  Similarly the gnomonic projection allows the intersection of
%   two geodesics to be determined quickly.
%
%   See also EQDAZIM_FWD, EQDAZIM_INV, CASSINI_FWD, CASSINI_INV,
%     TRANMERC_FWD, TRANMERC_INV, GNOMONIC_FWD, GNOMONIC_INV, UTM_FWD,
%     UTM_INV, DEFAULTELLIPSOID, ECC2FLAT, FLAT2ECC, GEODDISTANCE,
%     GEODRECKON.

% Copyright (c) Charles Karney (2012) <charles@karney.com>.
%
% This file was distributed with GeographicLib 1.29.

  help geodproj
end
