/**
 * \file GeodesicExact.hpp
 * \brief Header for GeographicLib::GeodesicExact class
 *
 * Copyright (c) Charles Karney (2012-2013) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * http://geographiclib.sourceforge.net/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_GEODESICEXACT_HPP)
#define GEOGRAPHICLIB_GEODESICEXACT_HPP 1

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/EllipticFunction.hpp>

#if !defined(GEOGRAPHICLIB_GEODESICEXACT_ORDER)
/**
 * The order of the expansions used by GeodesicExact.
 **********************************************************************/
#  define GEOGRAPHICLIB_GEODESICEXACT_ORDER 30
#endif

namespace GeographicLib {

  class GeodesicLineExact;

  /**
   * \brief Exact %Geodesic calculations
   *
   * The equations for geodesics on an ellipsoid can be expressed in terms of
   * incomplete elliptic integrals.  The Geodesic class expands these integrals
   * in a series in the flattening \e f and this provides an accurate solution
   * for \e f &isin; [-0.01, 0.01].  The GeodesicExact class computes the
   * ellitpic integrals directly and so provides a solution which is valid for
   * all \e f.  However, in practice, its use should be limited to about \e
   * b/\e a &isin; [0.01, 100] or \e f &isin; [-99, 0.99].
   *
   * For the WGS84 ellipsoid, these classes are 2--3 times \e slower than the
   * series solution and 2--3 times \e less \e accurate (because it's less easy
   * to control round-off errors with the elliptic integral formulation); i.e.,
   * the error is about 40 nm (40 nanometers) instead of 15 nm.  However the
   * error in the series solution scales as <i>f</i><sup>7</sup> while the
   * error in the elliptic integral solution depends weakly on \e f.  If the
   * quarter meridian distance is 10000 km and the ratio \e b/\e a = 1 &minus;
   * \e f is varied then the approximate maximum error (expressed as a
   * distance) is <pre>
   *       1 - f  error (nm)
   *       1/128     387
   *       1/64      345
   *       1/32      269
   *       1/16      210
   *       1/8       115
   *       1/4        69
   *       1/2        36
   *         1        15
   *         2        25
   *         4        96
   *         8       318
   *        16       985
   *        32      2352
   *        64      6008
   *       128     19024
   * </pre>
   *
   * The computation of the area in these classes is via a 30th order series.
   * This gives accurate results for \e b/\e a &isin; [1/2, 2]; the accuracy is
   * about 8 decimal digits for \e b/\e a &isin; [1/4, 4].
   *
   * See \ref geodellip for the formulation.  See the documentation on the
   * Geodesic class for additional information on the geodesics problems.
   *
   * Example of use:
   * \include example-GeodesicExact.cpp
   *
   * <a href="Geod.1.html">Geod</a> is a command-line utility providing access
   * to the functionality of GeodesicExact and GeodesicLineExact (via the -E
   * option).
   **********************************************************************/

  class GEOGRAPHICLIB_EXPORT GeodesicExact {
  private:
    typedef Math::real real;
    friend class GeodesicLineExact;
    static const int nC4_ = GEOGRAPHICLIB_GEODESICEXACT_ORDER;
    static const int nC4x_ = (nC4_ * (nC4_ + 1)) / 2;
    static const unsigned maxit1_ = 20;
    static const unsigned maxit2_ = maxit1_ +
      std::numeric_limits<real>::digits + 10;

    static const real tiny_;
    static const real tol0_;
    static const real tol1_;
    static const real tol2_;
    static const real tolb_;
    static const real xthresh_;

    enum captype {
      CAP_NONE = 0U,
      CAP_E    = 1U<<0,
      // Skip 1U<<1 for compatibility with Geodesic (not required)
      CAP_D    = 1U<<2,
      CAP_H    = 1U<<3,
      CAP_C4   = 1U<<4,
      CAP_ALL  = 0x1FU,
      OUT_ALL  = 0x7F80U,
    };

    static real CosSeries(real sinx, real cosx, const real c[], int n)
      throw();
    static inline real AngRound(real x) throw() {
      // The makes the smallest gap in x = 1/16 - nextafter(1/16, 0) = 1/2^57
      // for reals = 0.7 pm on the earth if x is an angle in degrees.  (This
      // is about 1000 times more resolution than we get with angles around 90
      // degrees.)  We use this to avoid having to deal with near singular
      // cases when x is non-zero but tiny (e.g., 1.0e-200).
      const real z = 1/real(16);
      volatile real y = std::abs(x);
      // The compiler mustn't "simplify" z - (z - y) to y
      y = y < z ? z - (z - y) : y;
      return x < 0 ? -y : y;
    }
    static inline void SinCosNorm(real& sinx, real& cosx) throw() {
      real r = Math::hypot(sinx, cosx);
      sinx /= r;
      cosx /= r;
    }
    static real Astroid(real x, real y) throw();

    real _a, _f, _f1, _e2, _ep2, _n, _b, _c2, _etol2;
    real _C4x[nC4x_];

    void Lengths(const EllipticFunction& E,
                 real sig12,
                 real ssig1, real csig1, real dn1,
                 real ssig2, real csig2, real dn2,
                 real cbet1, real cbet2,
                 real& s12s, real& m12a, real& m0,
                 bool scalep, real& M12, real& M21) const throw();
    real InverseStart(EllipticFunction& E,
                      real sbet1, real cbet1, real dn1,
                      real sbet2, real cbet2, real dn2,
                      real lam12,
                      real& salp1, real& calp1,
                      real& salp2, real& calp2, real& dnm) const throw();
    real Lambda12(real sbet1, real cbet1, real dn1,
                  real sbet2, real cbet2, real dn2,
                  real salp1, real calp1,
                  real& salp2, real& calp2, real& sig12,
                  real& ssig1, real& csig1, real& ssig2, real& csig2,
                  EllipticFunction& E,
                  real& omg12, bool diffp, real& dlam12)
      const throw();

    // These are Maxima generated functions to provide series approximations to
    // the integrals for the area.
    void C4coeff() throw();
    void C4f(real k2, real c[]) const throw();

  public:

    /**
     * Bit masks for what calculations to do.  These masks do double duty.
     * They signify to the GeodesicLineExact::GeodesicLineExact constructor and
     * to GeodesicExact::Line what capabilities should be included in the
     * GeodesicLineExact object.  They also specify which results to return in
     * the general routines GeodesicExact::GenDirect and
     * GeodesicExact::GenInverse routines.  GeodesicLineExact::mask is a
     * duplication of this enum.
     **********************************************************************/
    enum mask {
      /**
       * No capabilities, no output.
       * @hideinitializer
       **********************************************************************/
      NONE          = 0U,
      /**
       * Calculate latitude \e lat2.  (It's not necessary to include this as a
       * capability to GeodesicLineExact because this is included by default.)
       * @hideinitializer
       **********************************************************************/
      LATITUDE      = 1U<<7  | CAP_NONE,
      /**
       * Calculate longitude \e lon2.
       * @hideinitializer
       **********************************************************************/
      LONGITUDE     = 1U<<8  | CAP_H,
      /**
       * Calculate azimuths \e azi1 and \e azi2.  (It's not necessary to
       * include this as a capability to GeodesicLineExact because this is
       * included by default.)
       * @hideinitializer
       **********************************************************************/
      AZIMUTH       = 1U<<9  | CAP_NONE,
      /**
       * Calculate distance \e s12.
       * @hideinitializer
       **********************************************************************/
      DISTANCE      = 1U<<10 | CAP_E,
      /**
       * Allow distance \e s12 to be used as input in the direct geodesic
       * problem.
       * @hideinitializer
       **********************************************************************/
      DISTANCE_IN   = 1U<<11 | CAP_E,
      /**
       * Calculate reduced length \e m12.
       * @hideinitializer
       **********************************************************************/
      REDUCEDLENGTH = 1U<<12 | CAP_D,
      /**
       * Calculate geodesic scales \e M12 and \e M21.
       * @hideinitializer
       **********************************************************************/
      GEODESICSCALE = 1U<<13 | CAP_D,
      /**
       * Calculate area \e S12.
       * @hideinitializer
       **********************************************************************/
      AREA          = 1U<<14 | CAP_C4,
      /**
       * All capabilities, calculate everything.
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
     *   Negative \e f gives a prolate ellipsoid.  If \e f > 1, set flattening
     *   to 1/\e f.
     * @exception GeographicErr if \e a or (1 &minus; \e f ) \e a is not
     *   positive.
     **********************************************************************/
    GeodesicExact(real a, real f);
    ///@}

    /** \name Direct geodesic problem specified in terms of distance.
     **********************************************************************/
    ///@{
    /**
     * Perform the direct geodesic calculation where the length of the geodesic
     * is specified in terms of distance.
     *
     * @param[in] lat1 latitude of point 1 (degrees).
     * @param[in] lon1 longitude of point 1 (degrees).
     * @param[in] azi1 azimuth at point 1 (degrees).
     * @param[in] s12 distance between point 1 and point 2 (meters); it can be
     *   signed.
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
     * \e lat1 should be in the range [&minus;90&deg;, 90&deg;]; \e lon1 and \e
     * azi1 should be in the range [&minus;540&deg;, 540&deg;).  The values of
     * \e lon2 and \e azi2 returned are in the range [&minus;180&deg;,
     * 180&deg;).
     *
     * If either point is at a pole, the azimuth is defined by keeping the
     * longitude fixed, writing \e lat = &plusmn;(90&deg; &minus; &epsilon;),
     * and taking the limit &epsilon; &rarr; 0+.  An arc length greater that
     * 180&deg; signifies a geodesic which is not a shortest path.  (For a
     * prolate ellipsoid, an additional condition is necessary for a shortest
     * path: the longitudinal extent must not exceed of 180&deg;.)
     *
     * The following functions are overloaded versions of GeodesicExact::Direct
     * which omit some of the output parameters.  Note, however, that the arc
     * length is always computed and returned as the function value.
     **********************************************************************/
    Math::real Direct(real lat1, real lon1, real azi1, real s12,
                      real& lat2, real& lon2, real& azi2,
                      real& m12, real& M12, real& M21, real& S12)
      const throw() {
      real t;
      return GenDirect(lat1, lon1, azi1, false, s12,
                       LATITUDE | LONGITUDE | AZIMUTH |
                       REDUCEDLENGTH | GEODESICSCALE | AREA,
                       lat2, lon2, azi2, t, m12, M12, M21, S12);
    }

    /**
     * See the documentation for GeodesicExact::Direct.
     **********************************************************************/
    Math::real Direct(real lat1, real lon1, real azi1, real s12,
                      real& lat2, real& lon2)
      const throw() {
      real t;
      return GenDirect(lat1, lon1, azi1, false, s12,
                       LATITUDE | LONGITUDE,
                       lat2, lon2, t, t, t, t, t, t);
    }

    /**
     * See the documentation for GeodesicExact::Direct.
     **********************************************************************/
    Math::real Direct(real lat1, real lon1, real azi1, real s12,
                      real& lat2, real& lon2, real& azi2)
      const throw() {
      real t;
      return GenDirect(lat1, lon1, azi1, false, s12,
                       LATITUDE | LONGITUDE | AZIMUTH,
                       lat2, lon2, azi2, t, t, t, t, t);
    }

    /**
     * See the documentation for GeodesicExact::Direct.
     **********************************************************************/
    Math::real Direct(real lat1, real lon1, real azi1, real s12,
                      real& lat2, real& lon2, real& azi2, real& m12)
      const throw() {
      real t;
      return GenDirect(lat1, lon1, azi1, false, s12,
                       LATITUDE | LONGITUDE | AZIMUTH | REDUCEDLENGTH,
                       lat2, lon2, azi2, t, m12, t, t, t);
    }

    /**
     * See the documentation for GeodesicExact::Direct.
     **********************************************************************/
    Math::real Direct(real lat1, real lon1, real azi1, real s12,
                      real& lat2, real& lon2, real& azi2,
                      real& M12, real& M21)
      const throw() {
      real t;
      return GenDirect(lat1, lon1, azi1, false, s12,
                       LATITUDE | LONGITUDE | AZIMUTH | GEODESICSCALE,
                       lat2, lon2, azi2, t, t, M12, M21, t);
    }

    /**
     * See the documentation for GeodesicExact::Direct.
     **********************************************************************/
    Math::real Direct(real lat1, real lon1, real azi1, real s12,
                      real& lat2, real& lon2, real& azi2,
                      real& m12, real& M12, real& M21)
      const throw() {
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
     * Perform the direct geodesic calculation where the length of the geodesic
     * is specified in terms of arc length.
     *
     * @param[in] lat1 latitude of point 1 (degrees).
     * @param[in] lon1 longitude of point 1 (degrees).
     * @param[in] azi1 azimuth at point 1 (degrees).
     * @param[in] a12 arc length between point 1 and point 2 (degrees); it can
     *   be signed.
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
     * \e lat1 should be in the range [&minus;90&deg;, 90&deg;]; \e lon1 and \e
     * azi1 should be in the range [&minus;540&deg;, 540&deg;).  The values of
     * \e lon2 and \e azi2 returned are in the range [&minus;180&deg;,
     * 180&deg;).
     *
     * If either point is at a pole, the azimuth is defined by keeping the
     * longitude fixed, writing \e lat = &plusmn;(90&deg; &minus; &epsilon;),
     * and taking the limit &epsilon; &rarr; 0+.  An arc length greater that
     * 180&deg; signifies a geodesic which is not a shortest path.  (For a
     * prolate ellipsoid, an additional condition is necessary for a shortest
     * path: the longitudinal extent must not exceed of 180&deg;.)
     *
     * The following functions are overloaded versions of GeodesicExact::Direct
     * which omit some of the output parameters.
     **********************************************************************/
    void ArcDirect(real lat1, real lon1, real azi1, real a12,
                   real& lat2, real& lon2, real& azi2, real& s12,
                   real& m12, real& M12, real& M21, real& S12)
      const throw() {
      GenDirect(lat1, lon1, azi1, true, a12,
                LATITUDE | LONGITUDE | AZIMUTH | DISTANCE |
                REDUCEDLENGTH | GEODESICSCALE | AREA,
                lat2, lon2, azi2, s12, m12, M12, M21, S12);
    }

    /**
     * See the documentation for GeodesicExact::ArcDirect.
     **********************************************************************/
    void ArcDirect(real lat1, real lon1, real azi1, real a12,
                   real& lat2, real& lon2) const throw() {
      real t;
      GenDirect(lat1, lon1, azi1, true, a12,
                LATITUDE | LONGITUDE,
                lat2, lon2, t, t, t, t, t, t);
    }

    /**
     * See the documentation for GeodesicExact::ArcDirect.
     **********************************************************************/
    void ArcDirect(real lat1, real lon1, real azi1, real a12,
                   real& lat2, real& lon2, real& azi2) const throw() {
      real t;
      GenDirect(lat1, lon1, azi1, true, a12,
                LATITUDE | LONGITUDE | AZIMUTH,
                lat2, lon2, azi2, t, t, t, t, t);
    }

    /**
     * See the documentation for GeodesicExact::ArcDirect.
     **********************************************************************/
    void ArcDirect(real lat1, real lon1, real azi1, real a12,
                   real& lat2, real& lon2, real& azi2, real& s12)
      const throw() {
      real t;
      GenDirect(lat1, lon1, azi1, true, a12,
                LATITUDE | LONGITUDE | AZIMUTH | DISTANCE,
                lat2, lon2, azi2, s12, t, t, t, t);
    }

    /**
     * See the documentation for GeodesicExact::ArcDirect.
     **********************************************************************/
    void ArcDirect(real lat1, real lon1, real azi1, real a12,
                   real& lat2, real& lon2, real& azi2,
                   real& s12, real& m12) const throw() {
      real t;
      GenDirect(lat1, lon1, azi1, true, a12,
                LATITUDE | LONGITUDE | AZIMUTH | DISTANCE |
                REDUCEDLENGTH,
                lat2, lon2, azi2, s12, m12, t, t, t);
    }

    /**
     * See the documentation for GeodesicExact::ArcDirect.
     **********************************************************************/
    void ArcDirect(real lat1, real lon1, real azi1, real a12,
                   real& lat2, real& lon2, real& azi2, real& s12,
                   real& M12, real& M21) const throw() {
      real t;
      GenDirect(lat1, lon1, azi1, true, a12,
                LATITUDE | LONGITUDE | AZIMUTH | DISTANCE |
                GEODESICSCALE,
                lat2, lon2, azi2, s12, t, M12, M21, t);
    }

    /**
     * See the documentation for GeodesicExact::ArcDirect.
     **********************************************************************/
    void ArcDirect(real lat1, real lon1, real azi1, real a12,
                   real& lat2, real& lon2, real& azi2, real& s12,
                   real& m12, real& M12, real& M21) const throw() {
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
     * The general direct geodesic calculation.  GeodesicExact::Direct and
     * GeodesicExact::ArcDirect are defined in terms of this function.
     *
     * @param[in] lat1 latitude of point 1 (degrees).
     * @param[in] lon1 longitude of point 1 (degrees).
     * @param[in] azi1 azimuth at point 1 (degrees).
     * @param[in] arcmode boolean flag determining the meaning of the second
     *   parameter.
     * @param[in] s12_a12 if \e arcmode is false, this is the distance between
     *   point 1 and point 2 (meters); otherwise it is the arc length between
     *   point 1 and point 2 (degrees); it can be signed.
     * @param[in] outmask a bitor'ed combination of GeodesicExact::mask values
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
     * The GeodesicExact::mask values possible for \e outmask are
     * - \e outmask |= GeodesicExact::LATITUDE for the latitude \e lat2;
     * - \e outmask |= GeodesicExact::LONGITUDE for the latitude \e lon2;
     * - \e outmask |= GeodesicExact::AZIMUTH for the latitude \e azi2;
     * - \e outmask |= GeodesicExact::DISTANCE for the distance \e s12;
     * - \e outmask |= GeodesicExact::REDUCEDLENGTH for the reduced length \e
     *   m12;
     * - \e outmask |= GeodesicExact::GEODESICSCALE for the geodesic scales \e
     *   M12 and \e M21;
     * - \e outmask |= GeodesicExact::AREA for the area \e S12;
     * - \e outmask |= GeodesicExact::ALL for all of the above.
     * .
     * The function value \e a12 is always computed and returned and this
     * equals \e s12_a12 is \e arcmode is true.  If \e outmask includes
     * GeodesicExact::DISTANCE and \e arcmode is false, then \e s12 = \e
     * s12_a12.  It is not necessary to include GeodesicExact::DISTANCE_IN in
     * \e outmask; this is automatically included is \e arcmode is false.
     **********************************************************************/
    Math::real GenDirect(real lat1, real lon1, real azi1,
                         bool arcmode, real s12_a12, unsigned outmask,
                         real& lat2, real& lon2, real& azi2,
                         real& s12, real& m12, real& M12, real& M21,
                         real& S12) const throw();
    ///@}

    /** \name Inverse geodesic problem.
     **********************************************************************/
    ///@{
    /**
     * Perform the inverse geodesic calculation.
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
     * \e lat1 and \e lat2 should be in the range [&minus;90&deg;, 90&deg;]; \e
     * lon1 and \e lon2 should be in the range [&minus;540&deg;, 540&deg;).
     * The values of \e azi1 and \e azi2 returned are in the range
     * [&minus;180&deg;, 180&deg;).
     *
     * If either point is at a pole, the azimuth is defined by keeping the
     * longitude fixed, writing \e lat = &plusmn;(90&deg; &minus; &epsilon;),
     * and taking the limit &epsilon; &rarr; 0+.
     *
     * The following functions are overloaded versions of GeodesicExact::Inverse
     * which omit some of the output parameters.  Note, however, that the arc
     * length is always computed and returned as the function value.
     **********************************************************************/
    Math::real Inverse(real lat1, real lon1, real lat2, real lon2,
                       real& s12, real& azi1, real& azi2, real& m12,
                       real& M12, real& M21, real& S12) const throw() {
      return GenInverse(lat1, lon1, lat2, lon2,
                        DISTANCE | AZIMUTH |
                        REDUCEDLENGTH | GEODESICSCALE | AREA,
                        s12, azi1, azi2, m12, M12, M21, S12);
    }

    /**
     * See the documentation for GeodesicExact::Inverse.
     **********************************************************************/
    Math::real Inverse(real lat1, real lon1, real lat2, real lon2,
                       real& s12) const throw() {
      real t;
      return GenInverse(lat1, lon1, lat2, lon2,
                        DISTANCE,
                        s12, t, t, t, t, t, t);
    }

    /**
     * See the documentation for GeodesicExact::Inverse.
     **********************************************************************/
    Math::real Inverse(real lat1, real lon1, real lat2, real lon2,
                       real& azi1, real& azi2) const throw() {
      real t;
      return GenInverse(lat1, lon1, lat2, lon2,
                        AZIMUTH,
                        t, azi1, azi2, t, t, t, t);
    }

    /**
     * See the documentation for GeodesicExact::Inverse.
     **********************************************************************/
    Math::real Inverse(real lat1, real lon1, real lat2, real lon2,
                       real& s12, real& azi1, real& azi2)
      const throw() {
      real t;
      return GenInverse(lat1, lon1, lat2, lon2,
                        DISTANCE | AZIMUTH,
                        s12, azi1, azi2, t, t, t, t);
    }

    /**
     * See the documentation for GeodesicExact::Inverse.
     **********************************************************************/
    Math::real Inverse(real lat1, real lon1, real lat2, real lon2,
                       real& s12, real& azi1, real& azi2, real& m12)
      const throw() {
      real t;
      return GenInverse(lat1, lon1, lat2, lon2,
                        DISTANCE | AZIMUTH | REDUCEDLENGTH,
                        s12, azi1, azi2, m12, t, t, t);
    }

    /**
     * See the documentation for GeodesicExact::Inverse.
     **********************************************************************/
    Math::real Inverse(real lat1, real lon1, real lat2, real lon2,
                       real& s12, real& azi1, real& azi2,
                       real& M12, real& M21) const throw() {
      real t;
      return GenInverse(lat1, lon1, lat2, lon2,
                        DISTANCE | AZIMUTH | GEODESICSCALE,
                        s12, azi1, azi2, t, M12, M21, t);
    }

    /**
     * See the documentation for GeodesicExact::Inverse.
     **********************************************************************/
    Math::real Inverse(real lat1, real lon1, real lat2, real lon2,
                       real& s12, real& azi1, real& azi2, real& m12,
                       real& M12, real& M21) const throw() {
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
     * The general inverse geodesic calculation.  GeodesicExact::Inverse is
     * defined in terms of this function.
     *
     * @param[in] lat1 latitude of point 1 (degrees).
     * @param[in] lon1 longitude of point 1 (degrees).
     * @param[in] lat2 latitude of point 2 (degrees).
     * @param[in] lon2 longitude of point 2 (degrees).
     * @param[in] outmask a bitor'ed combination of GeodesicExact::mask values
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
     * The GeodesicExact::mask values possible for \e outmask are
     * - \e outmask |= GeodesicExact::DISTANCE for the distance \e s12;
     * - \e outmask |= GeodesicExact::AZIMUTH for the latitude \e azi2;
     * - \e outmask |= GeodesicExact::REDUCEDLENGTH for the reduced length \e
     *   m12;
     * - \e outmask |= GeodesicExact::GEODESICSCALE for the geodesic scales \e
     *   M12 and \e M21;
     * - \e outmask |= GeodesicExact::AREA for the area \e S12;
     * - \e outmask |= GeodesicExact::ALL for all of the above.
     * .
     * The arc length is always computed and returned as the function value.
     **********************************************************************/
    Math::real GenInverse(real lat1, real lon1, real lat2, real lon2,
                          unsigned outmask,
                          real& s12, real& azi1, real& azi2,
                          real& m12, real& M12, real& M21, real& S12)
      const throw();
    ///@}

    /** \name Interface to GeodesicLineExact.
     **********************************************************************/
    ///@{

    /**
     * Set up to compute several points on a single geodesic.
     *
     * @param[in] lat1 latitude of point 1 (degrees).
     * @param[in] lon1 longitude of point 1 (degrees).
     * @param[in] azi1 azimuth at point 1 (degrees).
     * @param[in] caps bitor'ed combination of GeodesicExact::mask values
     *   specifying the capabilities the GeodesicLineExact object should
     *   possess, i.e., which quantities can be returned in calls to
     *   GeodesicLineExact::Position.
     * @return a GeodesicLineExact object.
     *
     * \e lat1 should be in the range [&minus;90&deg;, 90&deg;]; \e lon1 and \e
     * azi1 should be in the range [&minus;540&deg;, 540&deg;).
     *
     * The GeodesicExact::mask values are
     * - \e caps |= GeodesicExact::LATITUDE for the latitude \e lat2; this is
     *   added automatically;
     * - \e caps |= GeodesicExact::LONGITUDE for the latitude \e lon2;
     * - \e caps |= GeodesicExact::AZIMUTH for the azimuth \e azi2; this is
     *   added automatically;
     * - \e caps |= GeodesicExact::DISTANCE for the distance \e s12;
     * - \e caps |= GeodesicExact::REDUCEDLENGTH for the reduced length \e m12;
     * - \e caps |= GeodesicExact::GEODESICSCALE for the geodesic scales \e M12
     *   and \e M21;
     * - \e caps |= GeodesicExact::AREA for the area \e S12;
     * - \e caps |= GeodesicExact::DISTANCE_IN permits the length of the
     *   geodesic to be given in terms of \e s12; without this capability the
     *   length can only be specified in terms of arc length;
     * - \e caps |= GeodesicExact::ALL for all of the above.
     * .
     * The default value of \e caps is GeodesicExact::ALL which turns on all
     * the capabilities.
     *
     * If the point is at a pole, the azimuth is defined by keeping \e lon1
     * fixed, writing \e lat1 = &plusmn;(90 &minus; &epsilon;), and taking the
     * limit &epsilon; &rarr; 0+.
     **********************************************************************/
    GeodesicLineExact Line(real lat1, real lon1, real azi1, unsigned caps = ALL)
      const throw();

    ///@}

    /** \name Inspector functions.
     **********************************************************************/
    ///@{

    /**
     * @return \e a the equatorial radius of the ellipsoid (meters).  This is
     *   the value used in the constructor.
     **********************************************************************/
    Math::real MajorRadius() const throw() { return _a; }

    /**
     * @return \e f the  flattening of the ellipsoid.  This is the
     *   value used in the constructor.
     **********************************************************************/
    Math::real Flattening() const throw() { return _f; }

    /// \cond SKIP
    /**
     * <b>DEPRECATED</b>
     * @return \e r the inverse flattening of the ellipsoid.
     **********************************************************************/
    Math::real InverseFlattening() const throw() { return 1/_f; }
    /// \endcond

    /**
     * @return total area of ellipsoid in meters<sup>2</sup>.  The area of a
     *   polygon encircling a pole can be found by adding
     *   GeodesicExact::EllipsoidArea()/2 to the sum of \e S12 for each side of
     *   the polygon.
     **********************************************************************/
    Math::real EllipsoidArea() const throw()
    { return 4 * Math::pi<real>() * _c2; }
    ///@}

    /**
     * A global instantiation of GeodesicExact with the parameters for the WGS84
     * ellipsoid.
     **********************************************************************/
    static const GeodesicExact WGS84;

  };

} // namespace GeographicLib

#endif  // GEOGRAPHICLIB_GEODESICEXACT_HPP
