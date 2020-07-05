/**
 * \file TransverseMercatorExact.hpp
 * \brief Header for GeographicLib::TransverseMercatorExact class
 *
 * Copyright (c) Charles Karney (2008-2016) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_TRANSVERSEMERCATOREXACT_HPP)
#define GEOGRAPHICLIB_TRANSVERSEMERCATOREXACT_HPP 1

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/EllipticFunction.hpp>

namespace GeographicLib {

  /**
   * \brief An exact implementation of the transverse Mercator projection
   *
   * Implementation of the Transverse Mercator Projection given in
   *  - L. P. Lee,
   *    <a href="https://doi.org/10.3138/X687-1574-4325-WM62"> Conformal
   *    Projections Based On Jacobian Elliptic Functions</a>, Part V of
   *    Conformal Projections Based on Elliptic Functions,
   *    (B. V. Gutsell, Toronto, 1976), 128pp.,
   *    ISBN: 0919870163
   *    (also appeared as:
   *    Monograph 16, Suppl. No. 1 to Canadian Cartographer, Vol 13).
   *  - C. F. F. Karney,
   *    <a href="https://doi.org/10.1007/s00190-011-0445-3">
   *    Transverse Mercator with an accuracy of a few nanometers,</a>
   *    J. Geodesy 85(8), 475--485 (Aug. 2011);
   *    preprint
   *    <a href="https://arxiv.org/abs/1002.1417">arXiv:1002.1417</a>.
   *
   * Lee gives the correct results for forward and reverse transformations
   * subject to the branch cut rules (see the description of the \e extendp
   * argument to the constructor).  The maximum error is about 8 nm (8
   * nanometers), ground distance, for the forward and reverse transformations.
   * The error in the convergence is 2 &times; 10<sup>&minus;15</sup>&quot;,
   * the relative error in the scale is 7 &times; 10<sup>&minus;12</sup>%%.
   * See Sec. 3 of
   * <a href="https://arxiv.org/abs/1002.1417">arXiv:1002.1417</a> for details.
   * The method is "exact" in the sense that the errors are close to the
   * round-off limit and that no changes are needed in the algorithms for them
   * to be used with reals of a higher precision.  Thus the errors using long
   * double (with a 64-bit fraction) are about 2000 times smaller than using
   * double (with a 53-bit fraction).
   *
   * This algorithm is about 4.5 times slower than the 6th-order Kr&uuml;ger
   * method, TransverseMercator, taking about 11 us for a combined forward and
   * reverse projection on a 2.66 GHz Intel machine (g++, version 4.3.0, -O3).
   *
   * The ellipsoid parameters and the central scale are set in the constructor.
   * The central meridian (which is a trivial shift of the longitude) is
   * specified as the \e lon0 argument of the TransverseMercatorExact::Forward
   * and TransverseMercatorExact::Reverse functions.  The latitude of origin is
   * taken to be the equator.  See the documentation on TransverseMercator for
   * how to include a false easting, false northing, or a latitude of origin.
   *
   * See <a href="https://geographiclib.sourceforge.io/tm-grid.kmz"
   * type="application/vnd.google-earth.kmz"> tm-grid.kmz</a>, for an
   * illustration of the transverse Mercator grid in Google Earth.
   *
   * This class also returns the meridian convergence \e gamma and scale \e k.
   * The meridian convergence is the bearing of grid north (the \e y axis)
   * measured clockwise from true north.
   *
   * See TransverseMercatorExact.cpp for more information on the
   * implementation.
   *
   * See \ref transversemercator for a discussion of this projection.
   *
   * Example of use:
   * \include example-TransverseMercatorExact.cpp
   *
   * <a href="TransverseMercatorProj.1.html">TransverseMercatorProj</a> is a
   * command-line utility providing access to the functionality of
   * TransverseMercator and TransverseMercatorExact.
   **********************************************************************/

  class GEOGRAPHICLIB_EXPORT TransverseMercatorExact {
  private:
    typedef Math::real real;
    static const int numit_ = 10;
    real tol_, tol2_, taytol_;
    real _a, _f, _k0, _mu, _mv, _e;
    bool _extendp;
    EllipticFunction _Eu, _Ev;

    void zeta(real u, real snu, real cnu, real dnu,
              real v, real snv, real cnv, real dnv,
              real& taup, real& lam) const;

    void dwdzeta(real u, real snu, real cnu, real dnu,
                 real v, real snv, real cnv, real dnv,
                 real& du, real& dv) const;

    bool zetainv0(real psi, real lam, real& u, real& v) const;
    void zetainv(real taup, real lam, real& u, real& v) const;

    void sigma(real u, real snu, real cnu, real dnu,
               real v, real snv, real cnv, real dnv,
               real& xi, real& eta) const;

    void dwdsigma(real u, real snu, real cnu, real dnu,
                  real v, real snv, real cnv, real dnv,
                  real& du, real& dv) const;

    bool sigmainv0(real xi, real eta, real& u, real& v) const;
    void sigmainv(real xi, real eta, real& u, real& v) const;

    void Scale(real tau, real lam,
               real snu, real cnu, real dnu,
               real snv, real cnv, real dnv,
               real& gamma, real& k) const;

  public:

    /**
     * Constructor for a ellipsoid with
     *
     * @param[in] a equatorial radius (meters).
     * @param[in] f flattening of ellipsoid.
     * @param[in] k0 central scale factor.
     * @param[in] extendp use extended domain.
     * @exception GeographicErr if \e a, \e f, or \e k0 is not positive.
     *
     * The transverse Mercator projection has a branch point singularity at \e
     * lat = 0 and \e lon &minus; \e lon0 = 90 (1 &minus; \e e) or (for
     * TransverseMercatorExact::UTM) x = 18381 km, y = 0m.  The \e extendp
     * argument governs where the branch cut is placed.  With \e extendp =
     * false, the "standard" convention is followed, namely the cut is placed
     * along \e x > 18381 km, \e y = 0m.  Forward can be called with any \e lat
     * and \e lon then produces the transformation shown in Lee, Fig 46.
     * Reverse analytically continues this in the &plusmn; \e x direction.  As
     * a consequence, Reverse may map multiple points to the same geographic
     * location; for example, for TransverseMercatorExact::UTM, \e x =
     * 22051449.037349 m, \e y = &minus;7131237.022729 m and \e x =
     * 29735142.378357 m, \e y = 4235043.607933 m both map to \e lat =
     * &minus;2&deg;, \e lon = 88&deg;.
     *
     * With \e extendp = true, the branch cut is moved to the lower left
     * quadrant.  The various symmetries of the transverse Mercator projection
     * can be used to explore the projection on any sheet.  In this mode the
     * domains of \e lat, \e lon, \e x, and \e y are restricted to
     * - the union of
     *   - \e lat in [0, 90] and \e lon &minus; \e lon0 in [0, 90]
     *   - \e lat in (-90, 0] and \e lon &minus; \e lon0 in [90 (1 &minus; \e
           e), 90]
     * - the union of
     *   - <i>x</i>/(\e k0 \e a) in [0, &infin;) and
     *     <i>y</i>/(\e k0 \e a) in [0, E(<i>e</i><sup>2</sup>)]
     *   - <i>x</i>/(\e k0 \e a) in [K(1 &minus; <i>e</i><sup>2</sup>) &minus;
     *     E(1 &minus; <i>e</i><sup>2</sup>), &infin;) and <i>y</i>/(\e k0 \e
     *     a) in (&minus;&infin;, 0]
     * .
     * See Sec. 5 of
     * <a href="https://arxiv.org/abs/1002.1417">arXiv:1002.1417</a> for a full
     * discussion of the treatment of the branch cut.
     *
     * The method will work for all ellipsoids used in terrestrial geodesy.
     * The method cannot be applied directly to the case of a sphere (\e f = 0)
     * because some the constants characterizing this method diverge in that
     * limit, and in practice, \e f should be larger than about
     * numeric_limits<real>::epsilon().  However, TransverseMercator treats the
     * sphere exactly.
     **********************************************************************/
    TransverseMercatorExact(real a, real f, real k0, bool extendp = false);

    /**
     * Forward projection, from geographic to transverse Mercator.
     *
     * @param[in] lon0 central meridian of the projection (degrees).
     * @param[in] lat latitude of point (degrees).
     * @param[in] lon longitude of point (degrees).
     * @param[out] x easting of point (meters).
     * @param[out] y northing of point (meters).
     * @param[out] gamma meridian convergence at point (degrees).
     * @param[out] k scale of projection at point.
     *
     * No false easting or northing is added. \e lat should be in the range
     * [&minus;90&deg;, 90&deg;].
     **********************************************************************/
    void Forward(real lon0, real lat, real lon,
                 real& x, real& y, real& gamma, real& k) const;

    /**
     * Reverse projection, from transverse Mercator to geographic.
     *
     * @param[in] lon0 central meridian of the projection (degrees).
     * @param[in] x easting of point (meters).
     * @param[in] y northing of point (meters).
     * @param[out] lat latitude of point (degrees).
     * @param[out] lon longitude of point (degrees).
     * @param[out] gamma meridian convergence at point (degrees).
     * @param[out] k scale of projection at point.
     *
     * No false easting or northing is added.  The value of \e lon returned is
     * in the range [&minus;180&deg;, 180&deg;].
     **********************************************************************/
    void Reverse(real lon0, real x, real y,
                 real& lat, real& lon, real& gamma, real& k) const;

    /**
     * TransverseMercatorExact::Forward without returning the convergence and
     * scale.
     **********************************************************************/
    void Forward(real lon0, real lat, real lon,
                 real& x, real& y) const {
      real gamma, k;
      Forward(lon0, lat, lon, x, y, gamma, k);
    }

    /**
     * TransverseMercatorExact::Reverse without returning the convergence and
     * scale.
     **********************************************************************/
    void Reverse(real lon0, real x, real y,
                 real& lat, real& lon) const {
      real gamma, k;
      Reverse(lon0, x, y, lat, lon, gamma, k);
    }

    /** \name Inspector functions
     **********************************************************************/
    ///@{
    /**
     * @return \e a the equatorial radius of the ellipsoid (meters).  This is
     *   the value used in the constructor.
     **********************************************************************/
    Math::real MajorRadius() const { return _a; }

    /**
     * @return \e f the flattening of the ellipsoid.  This is the value used in
     *   the constructor.
     **********************************************************************/
    Math::real Flattening() const { return _f; }

    /**
     * @return \e k0 central scale for the projection.  This is the value of \e
     *   k0 used in the constructor and is the scale on the central meridian.
     **********************************************************************/
    Math::real CentralScale() const { return _k0; }
    ///@}

    /**
     * A global instantiation of TransverseMercatorExact with the WGS84
     * ellipsoid and the UTM scale factor.  However, unlike UTM, no false
     * easting or northing is added.
     **********************************************************************/
    static const TransverseMercatorExact& UTM();
  };

} // namespace GeographicLib

#endif  // GEOGRAPHICLIB_TRANSVERSEMERCATOREXACT_HPP
