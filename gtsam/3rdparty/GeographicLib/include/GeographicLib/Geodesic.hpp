/**
 * \file Geodesic.hpp
 * \brief Header for GeographicLib::Geodesic class
 *
 * Copyright (c) Charles Karney (2009-2016) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_GEODESIC_HPP)
#define GEOGRAPHICLIB_GEODESIC_HPP 1

#include <GeographicLib/Constants.hpp>

#if !defined(GEOGRAPHICLIB_GEODESIC_ORDER)
/**
 * The order of the expansions used by Geodesic.
 * GEOGRAPHICLIB_GEODESIC_ORDER can be set to any integer in [3, 8].
 **********************************************************************/
#  define GEOGRAPHICLIB_GEODESIC_ORDER \
  (GEOGRAPHICLIB_PRECISION == 2 ? 6 : \
   (GEOGRAPHICLIB_PRECISION == 1 ? 3 : \
    (GEOGRAPHICLIB_PRECISION == 3 ? 7 : 8)))
#endif

namespace GeographicLib {

  class GeodesicLine;

  /**
   * \brief %Geodesic calculations
   *
   * The shortest path between two points on a ellipsoid at (\e lat1, \e lon1)
   * and (\e lat2, \e lon2) is called the geodesic.  Its length is \e s12 and
   * the geodesic from point 1 to point 2 has azimuths \e azi1 and \e azi2 at
   * the two end points.  (The azimuth is the heading measured clockwise from
   * north.  \e azi2 is the "forward" azimuth, i.e., the heading that takes you
   * beyond point 2 not back to point 1.)  In the figure below, latitude if
   * labeled &phi;, longitude &lambda; (with &lambda;<sub>12</sub> =
   * &lambda;<sub>2</sub> &minus; &lambda;<sub>1</sub>), and azimuth &alpha;.
   *
   * <img src="https://upload.wikimedia.org/wikipedia/commons/c/cb/Geodesic_problem_on_an_ellipsoid.svg" width=250 alt="spheroidal triangle">
   *
   * Given \e lat1, \e lon1, \e azi1, and \e s12, we can determine \e lat2, \e
   * lon2, and \e azi2.  This is the \e direct geodesic problem and its
   * solution is given by the function Geodesic::Direct.  (If \e s12 is
   * sufficiently large that the geodesic wraps more than halfway around the
   * earth, there will be another geodesic between the points with a smaller \e
   * s12.)
   *
   * Given \e lat1, \e lon1, \e lat2, and \e lon2, we can determine \e azi1, \e
   * azi2, and \e s12.  This is the \e inverse geodesic problem, whose solution
   * is given by Geodesic::Inverse.  Usually, the solution to the inverse
   * problem is unique.  In cases where there are multiple solutions (all with
   * the same \e s12, of course), all the solutions can be easily generated
   * once a particular solution is provided.
   *
   * The standard way of specifying the direct problem is the specify the
   * distance \e s12 to the second point.  However it is sometimes useful
   * instead to specify the arc length \e a12 (in degrees) on the auxiliary
   * sphere.  This is a mathematical construct used in solving the geodesic
   * problems.  The solution of the direct problem in this form is provided by
   * Geodesic::ArcDirect.  An arc length in excess of 180&deg; indicates that
   * the geodesic is not a shortest path.  In addition, the arc length between
   * an equatorial crossing and the next extremum of latitude for a geodesic is
   * 90&deg;.
   *
   * This class can also calculate several other quantities related to
   * geodesics.  These are:
   * - <i>reduced length</i>.  If we fix the first point and increase \e azi1
   *   by \e dazi1 (radians), the second point is displaced \e m12 \e dazi1 in
   *   the direction \e azi2 + 90&deg;.  The quantity \e m12 is called
   *   the "reduced length" and is symmetric under interchange of the two
   *   points.  On a curved surface the reduced length obeys a symmetry
   *   relation, \e m12 + \e m21 = 0.  On a flat surface, we have \e m12 = \e
   *   s12.  The ratio <i>s12</i>/\e m12 gives the azimuthal scale for an
   *   azimuthal equidistant projection.
   * - <i>geodesic scale</i>.  Consider a reference geodesic and a second
   *   geodesic parallel to this one at point 1 and separated by a small
   *   distance \e dt.  The separation of the two geodesics at point 2 is \e
   *   M12 \e dt where \e M12 is called the "geodesic scale".  \e M21 is
   *   defined similarly (with the geodesics being parallel at point 2).  On a
   *   flat surface, we have \e M12 = \e M21 = 1.  The quantity 1/\e M12 gives
   *   the scale of the Cassini-Soldner projection.
   * - <i>area</i>.  The area between the geodesic from point 1 to point 2 and
   *   the equation is represented by \e S12; it is the area, measured
   *   counter-clockwise, of the geodesic quadrilateral with corners
   *   (<i>lat1</i>,<i>lon1</i>), (0,<i>lon1</i>), (0,<i>lon2</i>), and
   *   (<i>lat2</i>,<i>lon2</i>).  It can be used to compute the area of any
   *   simple geodesic polygon.
   *
   * Overloaded versions of Geodesic::Direct, Geodesic::ArcDirect, and
   * Geodesic::Inverse allow these quantities to be returned.  In addition
   * there are general functions Geodesic::GenDirect, and Geodesic::GenInverse
   * which allow an arbitrary set of results to be computed.  The quantities \e
   * m12, \e M12, \e M21 which all specify the behavior of nearby geodesics
   * obey addition rules.  If points 1, 2, and 3 all lie on a single geodesic,
   * then the following rules hold:
   * - \e s13 = \e s12 + \e s23
   * - \e a13 = \e a12 + \e a23
   * - \e S13 = \e S12 + \e S23
   * - \e m13 = \e m12 \e M23 + \e m23 \e M21
   * - \e M13 = \e M12 \e M23 &minus; (1 &minus; \e M12 \e M21) \e m23 / \e m12
   * - \e M31 = \e M32 \e M21 &minus; (1 &minus; \e M23 \e M32) \e m12 / \e m23
   *
   * Additional functionality is provided by the GeodesicLine class, which
   * allows a sequence of points along a geodesic to be computed.
   *
   * The shortest distance returned by the solution of the inverse problem is
   * (obviously) uniquely defined.  However, in a few special cases there are
   * multiple azimuths which yield the same shortest distance.  Here is a
   * catalog of those cases:
   * - \e lat1 = &minus;\e lat2 (with neither point at a pole).  If \e azi1 =
   *   \e azi2, the geodesic is unique.  Otherwise there are two geodesics and
   *   the second one is obtained by setting [\e azi1, \e azi2] &rarr; [\e
   *   azi2, \e azi1], [\e M12, \e M21] &rarr; [\e M21, \e M12], \e S12 &rarr;
   *   &minus;\e S12.  (This occurs when the longitude difference is near
   *   &plusmn;180&deg; for oblate ellipsoids.)
   * - \e lon2 = \e lon1 &plusmn; 180&deg; (with neither point at a pole).  If
   *   \e azi1 = 0&deg; or &plusmn;180&deg;, the geodesic is unique.  Otherwise
   *   there are two geodesics and the second one is obtained by setting [\e
   *   azi1, \e azi2] &rarr; [&minus;\e azi1, &minus;\e azi2], \e S12 &rarr;
   *   &minus;\e S12.  (This occurs when \e lat2 is near &minus;\e lat1 for
   *   prolate ellipsoids.)
   * - Points 1 and 2 at opposite poles.  There are infinitely many geodesics
   *   which can be generated by setting [\e azi1, \e azi2] &rarr; [\e azi1, \e
   *   azi2] + [\e d, &minus;\e d], for arbitrary \e d.  (For spheres, this
   *   prescription applies when points 1 and 2 are antipodal.)
   * - \e s12 = 0 (coincident points).  There are infinitely many geodesics
   *   which can be generated by setting [\e azi1, \e azi2] &rarr;
   *   [\e azi1, \e azi2] + [\e d, \e d], for arbitrary \e d.
   *
   * The calculations are accurate to better than 15 nm (15 nanometers) for the
   * WGS84 ellipsoid.  See Sec. 9 of
   * <a href="https://arxiv.org/abs/1102.1215v1">arXiv:1102.1215v1</a> for
   * details.  The algorithms used by this class are based on series expansions
   * using the flattening \e f as a small parameter.  These are only accurate
   * for |<i>f</i>| &lt; 0.02; however reasonably accurate results will be
   * obtained for |<i>f</i>| &lt; 0.2.  Here is a table of the approximate
   * maximum error (expressed as a distance) for an ellipsoid with the same
   * equatorial radius as the WGS84 ellipsoid and different values of the
   * flattening.<pre>
   *     |f|      error
   *     0.01     25 nm
   *     0.02     30 nm
   *     0.05     10 um
   *     0.1     1.5 mm
   *     0.2     300 mm
   * </pre>
   * For very eccentric ellipsoids, use GeodesicExact instead.
   *
   * The algorithms are described in
   * - C. F. F. Karney,
   *   <a href="https://doi.org/10.1007/s00190-012-0578-z">
   *   Algorithms for geodesics</a>,
   *   J. Geodesy <b>87</b>, 43--55 (2013);
   *   DOI: <a href="https://doi.org/10.1007/s00190-012-0578-z">
   *   10.1007/s00190-012-0578-z</a>;
   *   addenda:
   *   <a href="https://geographiclib.sourceforge.io/geod-addenda.html">
   *   geod-addenda.html</a>.
   * .
   * For more information on geodesics see \ref geodesic.
   *
   * Example of use:
   * \include example-Geodesic.cpp
   *
   * <a href="GeodSolve.1.html">GeodSolve</a> is a command-line utility
   * providing access to the functionality of Geodesic and GeodesicLine.
   **********************************************************************/

  class GEOGRAPHICLIB_EXPORT Geodesic {
  private:
    typedef Math::real real;
    friend class GeodesicLine;
    static const int nA1_ = GEOGRAPHICLIB_GEODESIC_ORDER;
    static const int nC1_ = GEOGRAPHICLIB_GEODESIC_ORDER;
    static const int nC1p_ = GEOGRAPHICLIB_GEODESIC_ORDER;
    static const int nA2_ = GEOGRAPHICLIB_GEODESIC_ORDER;
    static const int nC2_ = GEOGRAPHICLIB_GEODESIC_ORDER;
    static const int nA3_ = GEOGRAPHICLIB_GEODESIC_ORDER;
    static const int nA3x_ = nA3_;
    static const int nC3_ = GEOGRAPHICLIB_GEODESIC_ORDER;
    static const int nC3x_ = (nC3_ * (nC3_ - 1)) / 2;
    static const int nC4_ = GEOGRAPHICLIB_GEODESIC_ORDER;
    static const int nC4x_ = (nC4_ * (nC4_ + 1)) / 2;
    // Size for temporary array
    // nC = max(max(nC1_, nC1p_, nC2_) + 1, max(nC3_, nC4_))
    static const int nC_ = GEOGRAPHICLIB_GEODESIC_ORDER + 1;
    static const unsigned maxit1_ = 20;
    unsigned maxit2_;
    real tiny_, tol0_, tol1_, tol2_, tolb_, xthresh_;

    enum captype {
      CAP_NONE = 0U,
      CAP_C1   = 1U<<0,
      CAP_C1p  = 1U<<1,
      CAP_C2   = 1U<<2,
      CAP_C3   = 1U<<3,
      CAP_C4   = 1U<<4,
      CAP_ALL  = 0x1FU,
      CAP_MASK = CAP_ALL,
      OUT_ALL  = 0x7F80U,
      OUT_MASK = 0xFF80U,       // Includes LONG_UNROLL
    };

    static real SinCosSeries(bool sinp,
                             real sinx, real cosx, const real c[], int n);
    static real Astroid(real x, real y);

    real _a, _f, _f1, _e2, _ep2, _n, _b, _c2, _etol2;
    real _A3x[nA3x_], _C3x[nC3x_], _C4x[nC4x_];

    void Lengths(real eps, real sig12,
                 real ssig1, real csig1, real dn1,
                 real ssig2, real csig2, real dn2,
                 real cbet1, real cbet2, unsigned outmask,
                 real& s12s, real& m12a, real& m0,
                 real& M12, real& M21, real Ca[]) const;
    real InverseStart(real sbet1, real cbet1, real dn1,
                      real sbet2, real cbet2, real dn2,
                      real lam12, real slam12, real clam12,
                      real& salp1, real& calp1,
                      real& salp2, real& calp2, real& dnm,
                      real Ca[]) const;
    real Lambda12(real sbet1, real cbet1, real dn1,
                  real sbet2, real cbet2, real dn2,
                  real salp1, real calp1, real slam120, real clam120,
                  real& salp2, real& calp2, real& sig12,
                  real& ssig1, real& csig1, real& ssig2, real& csig2,
                  real& eps, real& domg12,
                  bool diffp, real& dlam12, real Ca[]) const;
    real GenInverse(real lat1, real lon1, real lat2, real lon2,
                    unsigned outmask, real& s12,
                    real& salp1, real& calp1, real& salp2, real& calp2,
                    real& m12, real& M12, real& M21, real& S12) const;

    // These are Maxima generated functions to provide series approximations to
    // the integrals for the ellipsoidal geodesic.
    static real A1m1f(real eps);
    static void C1f(real eps, real c[]);
    static void C1pf(real eps, real c[]);
    static real A2m1f(real eps);
    static void C2f(real eps, real c[]);

    void A3coeff();
    real A3f(real eps) const;
    void C3coeff();
    void C3f(real eps, real c[]) const;
    void C4coeff();
    void C4f(real k2, real c[]) const;

  public:

    /**
     * Bit masks for what calculations to do.  These masks do double duty.
     * They signify to the GeodesicLine::GeodesicLine constructor and to
     * Geodesic::Line what capabilities should be included in the GeodesicLine
     * object.  They also specify which results to return in the general
     * routines Geodesic::GenDirect and Geodesic::GenInverse routines.
     * GeodesicLine::mask is a duplication of this enum.
     **********************************************************************/
    enum mask {
      /**
       * No capabilities, no output.
       * @hideinitializer
       **********************************************************************/
      NONE          = 0U,
      /**
       * Calculate latitude \e lat2.  (It's not necessary to include this as a
       * capability to GeodesicLine because this is included by default.)
       * @hideinitializer
       **********************************************************************/
      LATITUDE      = 1U<<7  | CAP_NONE,
      /**
       * Calculate longitude \e lon2.
       * @hideinitializer
       **********************************************************************/
      LONGITUDE     = 1U<<8  | CAP_C3,
      /**
       * Calculate azimuths \e azi1 and \e azi2.  (It's not necessary to
       * include this as a capability to GeodesicLine because this is included
       * by default.)
       * @hideinitializer
       **********************************************************************/
      AZIMUTH       = 1U<<9  | CAP_NONE,
      /**
       * Calculate distance \e s12.
       * @hideinitializer
       **********************************************************************/
      DISTANCE      = 1U<<10 | CAP_C1,
      /**
       * Allow distance \e s12 to be used as input in the direct geodesic
       * problem.
       * @hideinitializer
       **********************************************************************/
      DISTANCE_IN   = 1U<<11 | CAP_C1 | CAP_C1p,
      /**
       * Calculate reduced length \e m12.
       * @hideinitializer
       **********************************************************************/
      REDUCEDLENGTH = 1U<<12 | CAP_C1 | CAP_C2,
      /**
       * Calculate geodesic scales \e M12 and \e M21.
       * @hideinitializer
       **********************************************************************/
      GEODESICSCALE = 1U<<13 | CAP_C1 | CAP_C2,
      /**
       * Calculate area \e S12.
       * @hideinitializer
       **********************************************************************/
      AREA          = 1U<<14 | CAP_C4,
      /**
       * Unroll \e lon2 in the direct calculation.
       * @hideinitializer
       **********************************************************************/
      LONG_UNROLL   = 1U<<15,
      /**
       * All capabilities, calculate everything.  (LONG_UNROLL is not
       * included in this mask.)
       * @hideinitializer
       **********************************************************************/
      ALL           = OUT_ALL| CAP_ALL,
    };

    /** \name Constructor
     **********************************************************************/
    ///@{
    /**
     * Constructor for a ellipsoid with
     *
     * @param[in] a equatorial radius (meters).
     * @param[in] f flattening of ellipsoid.  Setting \e f = 0 gives a sphere.
     *   Negative \e f gives a prolate ellipsoid.
     * @exception GeographicErr if \e a or (1 &minus; \e f) \e a is not
     *   positive.
     **********************************************************************/
    Geodesic(real a, real f);
    ///@}

    /** \name Direct geodesic problem specified in terms of distance.
     **********************************************************************/
    ///@{
    /**
     * Solve the direct geodesic problem where the length of the geodesic
     * is specified in terms of distance.
     *
     * @param[in] lat1 latitude of point 1 (degrees).
     * @param[in] lon1 longitude of point 1 (degrees).
     * @param[in] azi1 azimuth at point 1 (degrees).
     * @param[in] s12 distance between point 1 and point 2 (meters); it can be
     *   negative.
     * @param[out] lat2 latitude of point 2 (degrees).
     * @param[out] lon2 longitude of point 2 (degrees).
     * @param[out] azi2 (forward) azimuth at point 2 (degrees).
     * @param[out] m12 reduced length of geodesic (meters).
     * @param[out] M12 geodesic scale of point 2 relative to point 1
     *   (dimensionless).
     * @param[out] M21 geodesic scale of point 1 relative to point 2
     *   (dimensionless).
     * @param[out] S12 area under the geodesic (meters<sup>2</sup>).
     * @return \e a12 arc length of between point 1 and point 2 (degrees).
     *
     * \e lat1 should be in the range [&minus;90&deg;, 90&deg;].  The values of
     * \e lon2 and \e azi2 returned are in the range [&minus;180&deg;,
     * 180&deg;].
     *
     * If either point is at a pole, the azimuth is defined by keeping the
     * longitude fixed, writing \e lat = &plusmn;(90&deg; &minus; &epsilon;),
     * and taking the limit &epsilon; &rarr; 0+.  An arc length greater that
     * 180&deg; signifies a geodesic which is not a shortest path.  (For a
     * prolate ellipsoid, an additional condition is necessary for a shortest
     * path: the longitudinal extent must not exceed of 180&deg;.)
     *
     * The following functions are overloaded versions of Geodesic::Direct
     * which omit some of the output parameters.  Note, however, that the arc
     * length is always computed and returned as the function value.
     **********************************************************************/
    Math::real Direct(real lat1, real lon1, real azi1, real s12,
                      real& lat2, real& lon2, real& azi2,
                      real& m12, real& M12, real& M21, real& S12)
      const {
      real t;
      return GenDirect(lat1, lon1, azi1, false, s12,
                       LATITUDE | LONGITUDE | AZIMUTH |
                       REDUCEDLENGTH | GEODESICSCALE | AREA,
                       lat2, lon2, azi2, t, m12, M12, M21, S12);
    }

    /**
     * See the documentation for Geodesic::Direct.
     **********************************************************************/
    Math::real Direct(real lat1, real lon1, real azi1, real s12,
                      real& lat2, real& lon2)
      const {
      real t;
      return GenDirect(lat1, lon1, azi1, false, s12,
                       LATITUDE | LONGITUDE,
                       lat2, lon2, t, t, t, t, t, t);
    }

    /**
     * See the documentation for Geodesic::Direct.
     **********************************************************************/
    Math::real Direct(real lat1, real lon1, real azi1, real s12,
                      real& lat2, real& lon2, real& azi2)
      const {
      real t;
      return GenDirect(lat1, lon1, azi1, false, s12,
                       LATITUDE | LONGITUDE | AZIMUTH,
                       lat2, lon2, azi2, t, t, t, t, t);
    }

    /**
     * See the documentation for Geodesic::Direct.
     **********************************************************************/
    Math::real Direct(real lat1, real lon1, real azi1, real s12,
                      real& lat2, real& lon2, real& azi2, real& m12)
      const {
      real t;
      return GenDirect(lat1, lon1, azi1, false, s12,
                       LATITUDE | LONGITUDE | AZIMUTH | REDUCEDLENGTH,
                       lat2, lon2, azi2, t, m12, t, t, t);
    }

    /**
     * See the documentation for Geodesic::Direct.
     **********************************************************************/
    Math::real Direct(real lat1, real lon1, real azi1, real s12,
                      real& lat2, real& lon2, real& azi2,
                      real& M12, real& M21)
      const {
      real t;
      return GenDirect(lat1, lon1, azi1, false, s12,
                       LATITUDE | LONGITUDE | AZIMUTH | GEODESICSCALE,
                       lat2, lon2, azi2, t, t, M12, M21, t);
    }

    /**
     * See the documentation for Geodesic::Direct.
     **********************************************************************/
    Math::real Direct(real lat1, real lon1, real azi1, real s12,
                      real& lat2, real& lon2, real& azi2,
                      real& m12, real& M12, real& M21)
      const {
      real t;
      return GenDirect(lat1, lon1, azi1, false, s12,
                       LATITUDE | LONGITUDE | AZIMUTH |
                       REDUCEDLENGTH | GEODESICSCALE,
                       lat2, lon2, azi2, t, m12, M12, M21, t);
    }
    ///@}

    /** \name Direct geodesic problem specified in terms of arc length.
     **********************************************************************/
    ///@{
    /**
     * Solve the direct geodesic problem where the length of the geodesic
     * is specified in terms of arc length.
     *
     * @param[in] lat1 latitude of point 1 (degrees).
     * @param[in] lon1 longitude of point 1 (degrees).
     * @param[in] azi1 azimuth at point 1 (degrees).
     * @param[in] a12 arc length between point 1 and point 2 (degrees); it can
     *   be negative.
     * @param[out] lat2 latitude of point 2 (degrees).
     * @param[out] lon2 longitude of point 2 (degrees).
     * @param[out] azi2 (forward) azimuth at point 2 (degrees).
     * @param[out] s12 distance between point 1 and point 2 (meters).
     * @param[out] m12 reduced length of geodesic (meters).
     * @param[out] M12 geodesic scale of point 2 relative to point 1
     *   (dimensionless).
     * @param[out] M21 geodesic scale of point 1 relative to point 2
     *   (dimensionless).
     * @param[out] S12 area under the geodesic (meters<sup>2</sup>).
     *
     * \e lat1 should be in the range [&minus;90&deg;, 90&deg;].  The values of
     * \e lon2 and \e azi2 returned are in the range [&minus;180&deg;,
     * 180&deg;].
     *
     * If either point is at a pole, the azimuth is defined by keeping the
     * longitude fixed, writing \e lat = &plusmn;(90&deg; &minus; &epsilon;),
     * and taking the limit &epsilon; &rarr; 0+.  An arc length greater that
     * 180&deg; signifies a geodesic which is not a shortest path.  (For a
     * prolate ellipsoid, an additional condition is necessary for a shortest
     * path: the longitudinal extent must not exceed of 180&deg;.)
     *
     * The following functions are overloaded versions of Geodesic::Direct
     * which omit some of the output parameters.
     **********************************************************************/
    void ArcDirect(real lat1, real lon1, real azi1, real a12,
                   real& lat2, real& lon2, real& azi2, real& s12,
                   real& m12, real& M12, real& M21, real& S12)
      const {
      GenDirect(lat1, lon1, azi1, true, a12,
                LATITUDE | LONGITUDE | AZIMUTH | DISTANCE |
                REDUCEDLENGTH | GEODESICSCALE | AREA,
                lat2, lon2, azi2, s12, m12, M12, M21, S12);
    }

    /**
     * See the documentation for Geodesic::ArcDirect.
     **********************************************************************/
    void ArcDirect(real lat1, real lon1, real azi1, real a12,
                   real& lat2, real& lon2) const {
      real t;
      GenDirect(lat1, lon1, azi1, true, a12,
                LATITUDE | LONGITUDE,
                lat2, lon2, t, t, t, t, t, t);
    }

    /**
     * See the documentation for Geodesic::ArcDirect.
     **********************************************************************/
    void ArcDirect(real lat1, real lon1, real azi1, real a12,
                   real& lat2, real& lon2, real& azi2) const {
      real t;
      GenDirect(lat1, lon1, azi1, true, a12,
                LATITUDE | LONGITUDE | AZIMUTH,
                lat2, lon2, azi2, t, t, t, t, t);
    }

    /**
     * See the documentation for Geodesic::ArcDirect.
     **********************************************************************/
    void ArcDirect(real lat1, real lon1, real azi1, real a12,
                   real& lat2, real& lon2, real& azi2, real& s12)
      const {
      real t;
      GenDirect(lat1, lon1, azi1, true, a12,
                LATITUDE | LONGITUDE | AZIMUTH | DISTANCE,
                lat2, lon2, azi2, s12, t, t, t, t);
    }

    /**
     * See the documentation for Geodesic::ArcDirect.
     **********************************************************************/
    void ArcDirect(real lat1, real lon1, real azi1, real a12,
                   real& lat2, real& lon2, real& azi2,
                   real& s12, real& m12) const {
      real t;
      GenDirect(lat1, lon1, azi1, true, a12,
                LATITUDE | LONGITUDE | AZIMUTH | DISTANCE |
                REDUCEDLENGTH,
                lat2, lon2, azi2, s12, m12, t, t, t);
    }

    /**
     * See the documentation for Geodesic::ArcDirect.
     **********************************************************************/
    void ArcDirect(real lat1, real lon1, real azi1, real a12,
                   real& lat2, real& lon2, real& azi2, real& s12,
                   real& M12, real& M21) const {
      real t;
      GenDirect(lat1, lon1, azi1, true, a12,
                LATITUDE | LONGITUDE | AZIMUTH | DISTANCE |
                GEODESICSCALE,
                lat2, lon2, azi2, s12, t, M12, M21, t);
    }

    /**
     * See the documentation for Geodesic::ArcDirect.
     **********************************************************************/
    void ArcDirect(real lat1, real lon1, real azi1, real a12,
                   real& lat2, real& lon2, real& azi2, real& s12,
                   real& m12, real& M12, real& M21) const {
      real t;
      GenDirect(lat1, lon1, azi1, true, a12,
                LATITUDE | LONGITUDE | AZIMUTH | DISTANCE |
                REDUCEDLENGTH | GEODESICSCALE,
                lat2, lon2, azi2, s12, m12, M12, M21, t);
    }
    ///@}

    /** \name General version of the direct geodesic solution.
     **********************************************************************/
    ///@{

    /**
     * The general direct geodesic problem.  Geodesic::Direct and
     * Geodesic::ArcDirect are defined in terms of this function.
     *
     * @param[in] lat1 latitude of point 1 (degrees).
     * @param[in] lon1 longitude of point 1 (degrees).
     * @param[in] azi1 azimuth at point 1 (degrees).
     * @param[in] arcmode boolean flag determining the meaning of the \e
     *   s12_a12.
     * @param[in] s12_a12 if \e arcmode is false, this is the distance between
     *   point 1 and point 2 (meters); otherwise it is the arc length between
     *   point 1 and point 2 (degrees); it can be negative.
     * @param[in] outmask a bitor'ed combination of Geodesic::mask values
     *   specifying which of the following parameters should be set.
     * @param[out] lat2 latitude of point 2 (degrees).
     * @param[out] lon2 longitude of point 2 (degrees).
     * @param[out] azi2 (forward) azimuth at point 2 (degrees).
     * @param[out] s12 distance between point 1 and point 2 (meters).
     * @param[out] m12 reduced length of geodesic (meters).
     * @param[out] M12 geodesic scale of point 2 relative to point 1
     *   (dimensionless).
     * @param[out] M21 geodesic scale of point 1 relative to point 2
     *   (dimensionless).
     * @param[out] S12 area under the geodesic (meters<sup>2</sup>).
     * @return \e a12 arc length of between point 1 and point 2 (degrees).
     *
     * The Geodesic::mask values possible for \e outmask are
     * - \e outmask |= Geodesic::LATITUDE for the latitude \e lat2;
     * - \e outmask |= Geodesic::LONGITUDE for the latitude \e lon2;
     * - \e outmask |= Geodesic::AZIMUTH for the latitude \e azi2;
     * - \e outmask |= Geodesic::DISTANCE for the distance \e s12;
     * - \e outmask |= Geodesic::REDUCEDLENGTH for the reduced length \e
     *   m12;
     * - \e outmask |= Geodesic::GEODESICSCALE for the geodesic scales \e
     *   M12 and \e M21;
     * - \e outmask |= Geodesic::AREA for the area \e S12;
     * - \e outmask |= Geodesic::ALL for all of the above;
     * - \e outmask |= Geodesic::LONG_UNROLL to unroll \e lon2 instead of
     *   wrapping it into the range [&minus;180&deg;, 180&deg;].
     * .
     * The function value \e a12 is always computed and returned and this
     * equals \e s12_a12 is \e arcmode is true.  If \e outmask includes
     * Geodesic::DISTANCE and \e arcmode is false, then \e s12 = \e s12_a12.
     * It is not necessary to include Geodesic::DISTANCE_IN in \e outmask; this
     * is automatically included is \e arcmode is false.
     *
     * With the Geodesic::LONG_UNROLL bit set, the quantity \e lon2 &minus; \e
     * lon1 indicates how many times and in what sense the geodesic encircles
     * the ellipsoid.
     **********************************************************************/
    Math::real GenDirect(real lat1, real lon1, real azi1,
                         bool arcmode, real s12_a12, unsigned outmask,
                         real& lat2, real& lon2, real& azi2,
                         real& s12, real& m12, real& M12, real& M21,
                         real& S12) const;
    ///@}

    /** \name Inverse geodesic problem.
     **********************************************************************/
    ///@{
    /**
     * Solve the inverse geodesic problem.
     *
     * @param[in] lat1 latitude of point 1 (degrees).
     * @param[in] lon1 longitude of point 1 (degrees).
     * @param[in] lat2 latitude of point 2 (degrees).
     * @param[in] lon2 longitude of point 2 (degrees).
     * @param[out] s12 distance between point 1 and point 2 (meters).
     * @param[out] azi1 azimuth at point 1 (degrees).
     * @param[out] azi2 (forward) azimuth at point 2 (degrees).
     * @param[out] m12 reduced length of geodesic (meters).
     * @param[out] M12 geodesic scale of point 2 relative to point 1
     *   (dimensionless).
     * @param[out] M21 geodesic scale of point 1 relative to point 2
     *   (dimensionless).
     * @param[out] S12 area under the geodesic (meters<sup>2</sup>).
     * @return \e a12 arc length of between point 1 and point 2 (degrees).
     *
     * \e lat1 and \e lat2 should be in the range [&minus;90&deg;, 90&deg;].
     * The values of \e azi1 and \e azi2 returned are in the range
     * [&minus;180&deg;, 180&deg;].
     *
     * If either point is at a pole, the azimuth is defined by keeping the
     * longitude fixed, writing \e lat = &plusmn;(90&deg; &minus; &epsilon;),
     * and taking the limit &epsilon; &rarr; 0+.
     *
     * The solution to the inverse problem is found using Newton's method.  If
     * this fails to converge (this is very unlikely in geodetic applications
     * but does occur for very eccentric ellipsoids), then the bisection method
     * is used to refine the solution.
     *
     * The following functions are overloaded versions of Geodesic::Inverse
     * which omit some of the output parameters.  Note, however, that the arc
     * length is always computed and returned as the function value.
     **********************************************************************/
    Math::real Inverse(real lat1, real lon1, real lat2, real lon2,
                       real& s12, real& azi1, real& azi2, real& m12,
                       real& M12, real& M21, real& S12) const {
      return GenInverse(lat1, lon1, lat2, lon2,
                        DISTANCE | AZIMUTH |
                        REDUCEDLENGTH | GEODESICSCALE | AREA,
                        s12, azi1, azi2, m12, M12, M21, S12);
    }

    /**
     * See the documentation for Geodesic::Inverse.
     **********************************************************************/
    Math::real Inverse(real lat1, real lon1, real lat2, real lon2,
                       real& s12) const {
      real t;
      return GenInverse(lat1, lon1, lat2, lon2,
                        DISTANCE,
                        s12, t, t, t, t, t, t);
    }

    /**
     * See the documentation for Geodesic::Inverse.
     **********************************************************************/
    Math::real Inverse(real lat1, real lon1, real lat2, real lon2,
                       real& azi1, real& azi2) const {
      real t;
      return GenInverse(lat1, lon1, lat2, lon2,
                        AZIMUTH,
                        t, azi1, azi2, t, t, t, t);
    }

    /**
     * See the documentation for Geodesic::Inverse.
     **********************************************************************/
    Math::real Inverse(real lat1, real lon1, real lat2, real lon2,
                       real& s12, real& azi1, real& azi2)
      const {
      real t;
      return GenInverse(lat1, lon1, lat2, lon2,
                        DISTANCE | AZIMUTH,
                        s12, azi1, azi2, t, t, t, t);
    }

    /**
     * See the documentation for Geodesic::Inverse.
     **********************************************************************/
    Math::real Inverse(real lat1, real lon1, real lat2, real lon2,
                       real& s12, real& azi1, real& azi2, real& m12)
      const {
      real t;
      return GenInverse(lat1, lon1, lat2, lon2,
                        DISTANCE | AZIMUTH | REDUCEDLENGTH,
                        s12, azi1, azi2, m12, t, t, t);
    }

    /**
     * See the documentation for Geodesic::Inverse.
     **********************************************************************/
    Math::real Inverse(real lat1, real lon1, real lat2, real lon2,
                       real& s12, real& azi1, real& azi2,
                       real& M12, real& M21) const {
      real t;
      return GenInverse(lat1, lon1, lat2, lon2,
                        DISTANCE | AZIMUTH | GEODESICSCALE,
                        s12, azi1, azi2, t, M12, M21, t);
    }

    /**
     * See the documentation for Geodesic::Inverse.
     **********************************************************************/
    Math::real Inverse(real lat1, real lon1, real lat2, real lon2,
                       real& s12, real& azi1, real& azi2, real& m12,
                       real& M12, real& M21) const {
      real t;
      return GenInverse(lat1, lon1, lat2, lon2,
                        DISTANCE | AZIMUTH |
                        REDUCEDLENGTH | GEODESICSCALE,
                        s12, azi1, azi2, m12, M12, M21, t);
    }
    ///@}

    /** \name General version of inverse geodesic solution.
     **********************************************************************/
    ///@{
    /**
     * The general inverse geodesic calculation.  Geodesic::Inverse is defined
     * in terms of this function.
     *
     * @param[in] lat1 latitude of point 1 (degrees).
     * @param[in] lon1 longitude of point 1 (degrees).
     * @param[in] lat2 latitude of point 2 (degrees).
     * @param[in] lon2 longitude of point 2 (degrees).
     * @param[in] outmask a bitor'ed combination of Geodesic::mask values
     *   specifying which of the following parameters should be set.
     * @param[out] s12 distance between point 1 and point 2 (meters).
     * @param[out] azi1 azimuth at point 1 (degrees).
     * @param[out] azi2 (forward) azimuth at point 2 (degrees).
     * @param[out] m12 reduced length of geodesic (meters).
     * @param[out] M12 geodesic scale of point 2 relative to point 1
     *   (dimensionless).
     * @param[out] M21 geodesic scale of point 1 relative to point 2
     *   (dimensionless).
     * @param[out] S12 area under the geodesic (meters<sup>2</sup>).
     * @return \e a12 arc length of between point 1 and point 2 (degrees).
     *
     * The Geodesic::mask values possible for \e outmask are
     * - \e outmask |= Geodesic::DISTANCE for the distance \e s12;
     * - \e outmask |= Geodesic::AZIMUTH for the latitude \e azi2;
     * - \e outmask |= Geodesic::REDUCEDLENGTH for the reduced length \e
     *   m12;
     * - \e outmask |= Geodesic::GEODESICSCALE for the geodesic scales \e
     *   M12 and \e M21;
     * - \e outmask |= Geodesic::AREA for the area \e S12;
     * - \e outmask |= Geodesic::ALL for all of the above.
     * .
     * The arc length is always computed and returned as the function value.
     **********************************************************************/
    Math::real GenInverse(real lat1, real lon1, real lat2, real lon2,
                          unsigned outmask,
                          real& s12, real& azi1, real& azi2,
                          real& m12, real& M12, real& M21, real& S12) const;
    ///@}

    /** \name Interface to GeodesicLine.
     **********************************************************************/
    ///@{

    /**
     * Set up to compute several points on a single geodesic.
     *
     * @param[in] lat1 latitude of point 1 (degrees).
     * @param[in] lon1 longitude of point 1 (degrees).
     * @param[in] azi1 azimuth at point 1 (degrees).
     * @param[in] caps bitor'ed combination of Geodesic::mask values
     *   specifying the capabilities the GeodesicLine object should possess,
     *   i.e., which quantities can be returned in calls to
     *   GeodesicLine::Position.
     * @return a GeodesicLine object.
     *
     * \e lat1 should be in the range [&minus;90&deg;, 90&deg;].
     *
     * The Geodesic::mask values are
     * - \e caps |= Geodesic::LATITUDE for the latitude \e lat2; this is
     *   added automatically;
     * - \e caps |= Geodesic::LONGITUDE for the latitude \e lon2;
     * - \e caps |= Geodesic::AZIMUTH for the azimuth \e azi2; this is
     *   added automatically;
     * - \e caps |= Geodesic::DISTANCE for the distance \e s12;
     * - \e caps |= Geodesic::REDUCEDLENGTH for the reduced length \e m12;
     * - \e caps |= Geodesic::GEODESICSCALE for the geodesic scales \e M12
     *   and \e M21;
     * - \e caps |= Geodesic::AREA for the area \e S12;
     * - \e caps |= Geodesic::DISTANCE_IN permits the length of the
     *   geodesic to be given in terms of \e s12; without this capability the
     *   length can only be specified in terms of arc length;
     * - \e caps |= Geodesic::ALL for all of the above.
     * .
     * The default value of \e caps is Geodesic::ALL.
     *
     * If the point is at a pole, the azimuth is defined by keeping \e lon1
     * fixed, writing \e lat1 = &plusmn;(90 &minus; &epsilon;), and taking the
     * limit &epsilon; &rarr; 0+.
     **********************************************************************/
    GeodesicLine Line(real lat1, real lon1, real azi1, unsigned caps = ALL)
      const;

    /**
     * Define a GeodesicLine in terms of the inverse geodesic problem.
     *
     * @param[in] lat1 latitude of point 1 (degrees).
     * @param[in] lon1 longitude of point 1 (degrees).
     * @param[in] lat2 latitude of point 2 (degrees).
     * @param[in] lon2 longitude of point 2 (degrees).
     * @param[in] caps bitor'ed combination of Geodesic::mask values
     *   specifying the capabilities the GeodesicLine object should possess,
     *   i.e., which quantities can be returned in calls to
     *   GeodesicLine::Position.
     * @return a GeodesicLine object.
     *
     * This function sets point 3 of the GeodesicLine to correspond to point 2
     * of the inverse geodesic problem.
     *
     * \e lat1 and \e lat2 should be in the range [&minus;90&deg;, 90&deg;].
     **********************************************************************/
    GeodesicLine InverseLine(real lat1, real lon1, real lat2, real lon2,
                             unsigned caps = ALL) const;

    /**
     * Define a GeodesicLine in terms of the direct geodesic problem specified
     * in terms of distance.
     *
     * @param[in] lat1 latitude of point 1 (degrees).
     * @param[in] lon1 longitude of point 1 (degrees).
     * @param[in] azi1 azimuth at point 1 (degrees).
     * @param[in] s12 distance between point 1 and point 2 (meters); it can be
     *   negative.
     * @param[in] caps bitor'ed combination of Geodesic::mask values
     *   specifying the capabilities the GeodesicLine object should possess,
     *   i.e., which quantities can be returned in calls to
     *   GeodesicLine::Position.
     * @return a GeodesicLine object.
     *
     * This function sets point 3 of the GeodesicLine to correspond to point 2
     * of the direct geodesic problem.
     *
     * \e lat1 should be in the range [&minus;90&deg;, 90&deg;].
     **********************************************************************/
    GeodesicLine DirectLine(real lat1, real lon1, real azi1, real s12,
                            unsigned caps = ALL) const;

    /**
     * Define a GeodesicLine in terms of the direct geodesic problem specified
     * in terms of arc length.
     *
     * @param[in] lat1 latitude of point 1 (degrees).
     * @param[in] lon1 longitude of point 1 (degrees).
     * @param[in] azi1 azimuth at point 1 (degrees).
     * @param[in] a12 arc length between point 1 and point 2 (degrees); it can
     *   be negative.
     * @param[in] caps bitor'ed combination of Geodesic::mask values
     *   specifying the capabilities the GeodesicLine object should possess,
     *   i.e., which quantities can be returned in calls to
     *   GeodesicLine::Position.
     * @return a GeodesicLine object.
     *
     * This function sets point 3 of the GeodesicLine to correspond to point 2
     * of the direct geodesic problem.
     *
     * \e lat1 should be in the range [&minus;90&deg;, 90&deg;].
     **********************************************************************/
    GeodesicLine ArcDirectLine(real lat1, real lon1, real azi1, real a12,
                               unsigned caps = ALL) const;

    /**
     * Define a GeodesicLine in terms of the direct geodesic problem specified
     * in terms of either distance or arc length.
     *
     * @param[in] lat1 latitude of point 1 (degrees).
     * @param[in] lon1 longitude of point 1 (degrees).
     * @param[in] azi1 azimuth at point 1 (degrees).
     * @param[in] arcmode boolean flag determining the meaning of the \e
     *   s12_a12.
     * @param[in] s12_a12 if \e arcmode is false, this is the distance between
     *   point 1 and point 2 (meters); otherwise it is the arc length between
     *   point 1 and point 2 (degrees); it can be negative.
     * @param[in] caps bitor'ed combination of Geodesic::mask values
     *   specifying the capabilities the GeodesicLine object should possess,
     *   i.e., which quantities can be returned in calls to
     *   GeodesicLine::Position.
     * @return a GeodesicLine object.
     *
     * This function sets point 3 of the GeodesicLine to correspond to point 2
     * of the direct geodesic problem.
     *
     * \e lat1 should be in the range [&minus;90&deg;, 90&deg;].
     **********************************************************************/
    GeodesicLine GenDirectLine(real lat1, real lon1, real azi1,
                               bool arcmode, real s12_a12,
                               unsigned caps = ALL) const;
    ///@}

    /** \name Inspector functions.
     **********************************************************************/
    ///@{

    /**
     * @return \e a the equatorial radius of the ellipsoid (meters).  This is
     *   the value used in the constructor.
     **********************************************************************/
    Math::real MajorRadius() const { return _a; }

    /**
     * @return \e f the  flattening of the ellipsoid.  This is the
     *   value used in the constructor.
     **********************************************************************/
    Math::real Flattening() const { return _f; }

    /**
     * @return total area of ellipsoid in meters<sup>2</sup>.  The area of a
     *   polygon encircling a pole can be found by adding
     *   Geodesic::EllipsoidArea()/2 to the sum of \e S12 for each side of the
     *   polygon.
     **********************************************************************/
    Math::real EllipsoidArea() const
    { return 4 * Math::pi() * _c2; }
    ///@}

    /**
     * A global instantiation of Geodesic with the parameters for the WGS84
     * ellipsoid.
     **********************************************************************/
    static const Geodesic& WGS84();

  };

} // namespace GeographicLib

#endif  // GEOGRAPHICLIB_GEODESIC_HPP
