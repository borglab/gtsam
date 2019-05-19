function gedoc
%GEDOC  Great ellipses on an ellipsoid of revolution
%
%   The two routines GEDISTANCE and GERECKON solve the inverse and direct
%   problems for great ellipses on the surface of an ellipsoid of
%   revolution.  For more information, see
%
%     https://geographiclib.sourceforge.io/html/greatellipse.html
%
%   Great ellipses are sometimes proposed as alternatives to computing
%   ellipsoidal geodesics.  However geodesic calculations are easy to
%   perform using geoddistance and geodreckon, and these should normally be
%   used instead of gedistance and gereckon.  For a discussion, see
%
%     https://geographiclib.sourceforge.io/html/greatellipse.html#gevsgeodesic
%
%   The method involves stretching the ellipse along the axis until it
%   becomes a sphere, solving the corresponding great circle problem on the
%   sphere and mapping the results back to the ellipsoid.  For details,
%   see
%
%     https://geographiclib.sourceforge.io/html/greatellipse.html#geformulation
%
%   Consider two points on the ellipsoid at (lat1, lon1) and (lat2, lon2).
%   The plane containing these points and the center of the ellipsoid
%   intersects the ellipsoid on a great ellipse.  The length of the shorter
%   portion of the great ellipse between the two points is s12 and the
%   great ellipse from point 1 to point 2 has forward azimuths azi1 and
%   azi2 at the two end points.
%
%   Two great ellipse problems can be considered:
%     * the direct problem -- given lat1, lon1, s12, and azi1, determine
%       lat2, lon2, and azi2.  This is solved by gereckon.
%     * the inverse problem -- given lat1, lon1, lat2, lon2, determine s12,
%       azi1, and azi2.  This is solved by gedistance.
%
%   The routines also optionally calculate S12 ,the area between the great
%   ellipse from point 1 to point 2 and the equator; i.e., it is the area,
%   measured counter-clockwise, of the quadrilateral with corners
%   (lat1,lon1), (0,lon1), (0,lon2), and (lat2,lon2).  It is given in
%   meters^2.
%
%   The parameters of the ellipsoid are specified by the optional ellipsoid
%   argument to the routines.  This is a two-element vector of the form
%   [a,e], where a is the equatorial radius, e is the eccentricity e =
%   sqrt(a^2-b^2)/a, and b is the polar semi-axis.  Typically, a and b are
%   measured in meters and the distances returned by the routines are then
%   in meters.  However, other units can be employed.  If ellipsoid is
%   omitted, then the WGS84 ellipsoid (more precisely, the value returned
%   by defaultellipsoid) is assumed [6378137, 0.0818191908426215]
%   corresponding to a = 6378137 meters and a flattening f = (a-b)/a =
%   1/298.257223563.  The flattening and eccentricity are related by
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
%     * The distance s12 is unrestricted.  This allows great ellipses to
%       wrap around the ellipsoid.
%     * The equatorial radius, a, must be positive.
%     * The eccentricity, e, should be satisfy abs(e) < 0.2 in order to
%       retain full accuracy (this corresponds to flattenings satisfying
%       abs(f) <= 1/50, approximately).  This condition holds for most
%       applications in geodesy.
%
%   Larger values of e can be used with a corresponding drop in accuracy.
%   The following table gives the approximate maximum error in gedistance
%   and gereckon (expressed as a distance) for an ellipsoid with the same
%   equatorial radius as the WGS84 ellipsoid and different values of the
%   flattening (nm means nanometer and um means micrometer).
%
%        |f|     error
%        0.01    25 nm
%        0.02    30 nm
%        0.05    10 um
%        0.1    1.5 mm
%        0.2    300 mm
%
%   In order to compute intermediate points on a great ellipse, proceed as
%   in the following example which plots the track from Sydney to
%   Valparaiso and computes the deviation from the corresponding geodesic.
%
%       % 1 = Sydney, 2 = Valparaiso
%       lat1 = -33.83; lon1 = 151.29;
%       lat2 = -33.02; lon2 = -71.64;
%       [s12g, azi1g] = geoddistance(lat1, lon1, lat2, lon2);
%       [s12e, azi1e] =   gedistance(lat1, lon1, lat2, lon2);
%       fprintf('Difference in lengths = %.1f m\n', s12e - s12g);
%       [latg, long] = geodreckon(lat1, lon1, s12g * [0:100]/100, azi1g);
%       [late, lone] =   gereckon(lat1, lon1, s12e * [0:100]/100, azi1e);
%       plot(long+360*(long<0), latg, lone+360*(lone<0), late);
%       legend('geodesic', 'great ellipse', 'Location', 'SouthEast');
%       title('Sydney to Valparaiso');
%       xlabel('longitude'); ylabel('latitude');
%       fprintf('Maximum separation = %.1f km\n', ...
%               max(geoddistance(latg, long, late, lone))/1000);
%
%   The restriction on e above arises because the meridian distance is
%   given as a series expansion in the third flattening.  The exact
%   distance (valid for any e) can be expressed in terms of the elliptic
%   integral of the second kind.
%
%   See also GEDISTANCE, GERECKON, DEFAULTELLIPSOID, ECC2FLAT, FLAT2ECC,
%     GEODDISTANCE, GEODRECKON.

% Copyright (c) Charles Karney (2014-2017) <charles@karney.com>.

  help gedoc
end
