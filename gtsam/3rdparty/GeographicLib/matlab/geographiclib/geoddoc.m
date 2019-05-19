function geoddoc
%GEODDOC  Geodesics on an ellipsoid of revolution
%
%   The routines geoddistance, geodreckon, and geodarea solve various
%   problems involving geodesics on the surface of an ellipsoid of
%   revolution.  These are based on the paper
%
%     C. F. F. Karney, Algorithms for geodesics,
%     J. Geodesy 87, 43-55 (2013);
%     https://doi.org/10.1007/s00190-012-0578-z
%     Addenda: https://geographiclib.sourceforge.io/geod-addenda.html
%
%   which, in turn, is based on the classic solution of the geodesic
%   problems pioneered by Legendre (1806), Bessel (1825), and Helmert
%   (1880).  Links for these and other original papers on geodesics are
%   given in
%
%     https://geographiclib.sourceforge.io/geodesic-papers/biblio.html
%
%   The shortest path between two points on the ellipsoid at (lat1, lon1)
%   and (lat2, lon2) is called the geodesic.  Its length is s12 and the
%   geodesic from point 1 to point 2 has forward azimuths azi1 and azi2 at
%   the two end points.
%
%   Traditionally two geodesic problems are considered:
%     * the direct problem -- given lat1, lon1, s12, and azi1, determine
%       lat2, lon2, and azi2.  This is solved by geodreckon.
%     * the inverse problem -- given lat1, lon1, lat2, lon2, determine s12,
%       azi1, and azi2.  This is solved by geoddistance.
%   In addition, geodarea computes the area of an ellipsoidal polygon
%   where the edges are defined as shortest geodesics.
%
%   The parameters of the ellipsoid are specified by the optional ellipsoid
%   argument to the routines.  This is a two-element vector of the form
%   [a,e], where a is the equatorial radius, e is the eccentricity e =
%   sqrt(a^2-b^2)/a, and b is the polar semi-axis.  Typically, a and b are
%   measured in meters and the linear and area quantities returned by the
%   routines are then in meters and meters^2.  However, other units can be
%   employed.  If ELLIPSOID is omitted, then the WGS84 ellipsoid (more
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
%   The routines also calculate several other quantities of interest
%     * S12 is the area between the geodesic from point 1 to point 2 and
%       the equator; i.e., it is the area, measured counter-clockwise, of
%       the quadrilateral with corners (lat1,lon1), (0,lon1), (0,lon2), and
%       (lat2,lon2).  It is given in meters^2.
%     * m12, the reduced length of the geodesic is defined such that if the
%       initial azimuth is perturbed by dazi1 (radians) then the second
%       point is displaced by m12 dazi1 in the direction perpendicular to
%       the geodesic.  m12 is given in meters.  On a curved surface the
%       reduced length obeys a symmetry relation, m12 + m21 = 0.  On a flat
%       surface, we have m12 = s12.
%     * M12 and M21 are geodesic scales.  If two geodesics are parallel at
%       point 1 and separated by a small distance dt, then they are
%       separated by a distance M12 dt at point 2.  M21 is defined
%       similarly (with the geodesics being parallel to one another at
%       point 2).  M12 and M21 are dimensionless quantities.  On a flat
%       surface, we have M12 = M21 = 1.
%     * a12 is the arc length on the auxiliary sphere.  This is a construct
%       for converting the problem to one in spherical trigonometry.  a12
%       is measured in degrees.  The spherical arc length from one equator
%       crossing to the next is always 180 degrees.
%
%   If points 1, 2, and 3 lie on a single geodesic, then the following
%   addition rules hold:
%     * s13 = s12 + s23
%     * a13 = a12 + a23
%     * S13 = S12 + S23
%     * m13 = m12*M23 + m23*M21
%     * M13 = M12*M23 - (1 - M12*M21) * m23/m12
%     * M31 = M32*M21 - (1 - M23*M32) * m12/m23
%
%   Restrictions on the inputs:
%     * All latitudes must lie in [-90, 90].
%     * The distance s12 is unrestricted.  This allows geodesics to wrap
%       around the ellipsoid.  Such geodesics are no longer shortest paths.
%       However they retain the property that they are the straightest
%       curves on the surface.
%     * Similarly, the spherical arc length, a12, is unrestricted.
%     * The equatorial radius, a, must be positive.
%     * The eccentricity, e, should satisfy abs(e) < 0.2 in order to
%       retain full accuracy (this corresponds to flattenings satisfying
%       abs(f) <= 1/50, approximately).  This condition holds for most
%       applications in geodesy.
%
%    Larger values of e can be used with a corresponding drop in accuracy.
%    The following table gives the approximate maximum error in
%    geoddistance and geodreckon (expressed as a distance) for an ellipsoid
%    with the same equatorial radius as the WGS84 ellipsoid and different
%    values of the flattening (nm means nanometer and um means micrometer).
%
%         |f|     error
%         0.01    25 nm
%         0.02    30 nm
%         0.05    10 um
%         0.1    1.5 mm
%         0.2    300 mm
%
%   The shortest distance returned by GEODDISTANCE is (obviously) uniquely
%   defined.  However, in a few special cases there are multiple azimuths
%   which yield the same shortest distance.  Here is a catalog of those
%   cases:
%     * lat1 = -lat2 (with neither point at a pole).  If azi1 = azi2, the
%       geodesic is unique.  Otherwise there are two geodesics and the
%       second one is obtained by setting [azi1,azi2] = [azi2,azi1],
%       [M12,M21] = [M21,M12], S12 = -S12.  (This occurs when the longitude
%       difference is near +/-180 for oblate ellipsoids.)
%     * lon2 = lon1 +/- 180 (with neither point at a pole).  If azi1 = 0 or
%       +/-180, the geodesic is unique.  Otherwise there are two geodesics
%       and the second one is obtained by setting [azi1,azi2] =
%       [-azi1,-azi2], S12 = -S12.  (This occurs when lat2 is near -lat1
%       for prolate ellipsoids.)
%     * Points 1 and 2 at opposite poles.  There are infinitely many
%       geodesics which can be generated by setting [azi1,azi2] =
%       [azi1,azi2] + [d,-d], for arbitrary d.  (For spheres, this
%       prescription applies when points 1 and 2 are antipodal.)
%     * s12 = 0 (coincident points).  There are infinitely many geodesics
%       which can be generated by setting [azi1,azi2] = [azi1,azi2] +
%       [d,d], for arbitrary d.
%
%   In order to compute intermediate points on a geodesic, proceed as in
%   the following example which plots the track from JFK Airport to
%   Singapore Changi Airport.
%
%       lat1 = 40.64; lon1 = -73.78;
%       lat2 =  1.36; lon2 = 103.99;
%       [s12,  azi1] = geoddistance(lat1, lon1, lat2, lon2);
%       [lats, lons] = geodreckon(lat1, lon1, s12 * [0:100]/100, azi1);
%       plot(lons, lats);
%
%   The restriction on e above arises because the formulation is in terms
%   of series expansions in e^2.  The exact solutions (valid for any e) can
%   be expressed in terms of elliptic integrals.  These are provided by the
%   C++ classes GeodesicExact and GeodesicLineExact.
%
%   The routines duplicate some of the functionality of the distance,
%   reckon, and areaint functions in the MATLAB mapping toolbox.  The major
%   improvements offered by geoddistance, geodreckon, and geodarea are
%
%     * The routines are accurate to round off for abs(e) < 0.2.  For
%       example, for the WGS84 ellipsoid, the error in the distance
%       returned by geoddistance is less then 15 nanometers.
%     * The routines work for prolate (as well as oblate) ellipsoids.
%     * geoddistance converges for all inputs.
%     * Differential and integral properties of the geodesics are computed.
%     * geodarea is accurate regardless of the length of the edges of the
%       polygon.
%
%   See also GEODDISTANCE, GEODRECKON, GEODAREA,
%     DEFAULTELLIPSOID, ECC2FLAT, FLAT2ECC.

% Copyright (c) Charles Karney (2012-2017) <charles@karney.com>.

  help geoddoc
end
