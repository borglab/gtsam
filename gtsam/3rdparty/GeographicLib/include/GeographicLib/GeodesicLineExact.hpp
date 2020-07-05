/**
 * \file GeodesicLineExact.hpp
 * \brief Header for GeographicLib::GeodesicLineExact class
 *
 * Copyright (c) Charles Karney (2012-2016) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_GEODESICLINEEXACT_HPP)
#define GEOGRAPHICLIB_GEODESICLINEEXACT_HPP 1

#include <GeographicLib/Constants.hpp>
#include <GeographicLib/GeodesicExact.hpp>
#include <GeographicLib/EllipticFunction.hpp>

namespace GeographicLib {

  /**
   * \brief An exact geodesic line
   *
   * GeodesicLineExact facilitates the determination of a series of points on a
   * single geodesic.  This is a companion to the GeodesicExact class.  For
   * additional information on this class see the documentation on the
   * GeodesicLine class.
   *
   * Example of use:
   * \include example-GeodesicLineExact.cpp
   *
   * <a href="GeodSolve.1.html">GeodSolve</a> is a command-line utility
   * providing access to the functionality of GeodesicExact and
   * GeodesicLineExact (via the -E option).
   **********************************************************************/

  class GEOGRAPHICLIB_EXPORT GeodesicLineExact {
  private:
    typedef Math::real real;
    friend class GeodesicExact;
    static const int nC4_ = GeodesicExact::nC4_;

    real tiny_;
    real _lat1, _lon1, _azi1;
    real _a, _f, _b, _c2, _f1, _e2, _salp0, _calp0, _k2,
      _salp1, _calp1, _ssig1, _csig1, _dn1, _stau1, _ctau1,
      _somg1, _comg1, _cchi1,
      _A4, _B41, _E0, _D0, _H0, _E1, _D1, _H1;
    real _a13, _s13;
    real _C4a[nC4_];            // all the elements of _C4a are used
    EllipticFunction _E;
    unsigned _caps;

    void LineInit(const GeodesicExact& g,
                  real lat1, real lon1,
                  real azi1, real salp1, real calp1,
                  unsigned caps);
    GeodesicLineExact(const GeodesicExact& g,
                      real lat1, real lon1,
                      real azi1, real salp1, real calp1,
                      unsigned caps, bool arcmode, real s13_a13);

    enum captype {
      CAP_NONE = GeodesicExact::CAP_NONE,
      CAP_E    = GeodesicExact::CAP_E,
      CAP_D    = GeodesicExact::CAP_D,
      CAP_H    = GeodesicExact::CAP_H,
      CAP_C4   = GeodesicExact::CAP_C4,
      CAP_ALL  = GeodesicExact::CAP_ALL,
      CAP_MASK = GeodesicExact::CAP_MASK,
      OUT_ALL  = GeodesicExact::OUT_ALL,
      OUT_MASK = GeodesicExact::OUT_MASK,
    };
  public:

    /**
     * Bit masks for what calculations to do.  They signify to the
     * GeodesicLineExact::GeodesicLineExact constructor and to
     * GeodesicExact::Line what capabilities should be included in the
     * GeodesicLineExact object.  This is merely a duplication of
     * GeodesicExact::mask.
     **********************************************************************/
    enum mask {
      /**
       * No capabilities, no output.
       * @hideinitializer
       **********************************************************************/
      NONE          = GeodesicExact::NONE,
      /**
       * Calculate latitude \e lat2.  (It's not necessary to include this as a
       * capability to GeodesicLineExact because this is included by default.)
       * @hideinitializer
       **********************************************************************/
      LATITUDE      = GeodesicExact::LATITUDE,
      /**
       * Calculate longitude \e lon2.
       * @hideinitializer
       **********************************************************************/
      LONGITUDE     = GeodesicExact::LONGITUDE,
      /**
       * Calculate azimuths \e azi1 and \e azi2.  (It's not necessary to
       * include this as a capability to GeodesicLineExact because this is
       * included by default.)
       * @hideinitializer
       **********************************************************************/
      AZIMUTH       = GeodesicExact::AZIMUTH,
      /**
       * Calculate distance \e s12.
       * @hideinitializer
       **********************************************************************/
      DISTANCE      = GeodesicExact::DISTANCE,
      /**
       * Allow distance \e s12 to be used as input in the direct geodesic
       * problem.
       * @hideinitializer
       **********************************************************************/
      DISTANCE_IN   = GeodesicExact::DISTANCE_IN,
      /**
       * Calculate reduced length \e m12.
       * @hideinitializer
       **********************************************************************/
      REDUCEDLENGTH = GeodesicExact::REDUCEDLENGTH,
      /**
       * Calculate geodesic scales \e M12 and \e M21.
       * @hideinitializer
       **********************************************************************/
      GEODESICSCALE = GeodesicExact::GEODESICSCALE,
      /**
       * Calculate area \e S12.
       * @hideinitializer
       **********************************************************************/
      AREA          = GeodesicExact::AREA,
      /**
       * Unroll \e lon2 in the direct calculation.
       * @hideinitializer
       **********************************************************************/
      LONG_UNROLL = GeodesicExact::LONG_UNROLL,
      /**
       * All capabilities, calculate everything.  (LONG_UNROLL is not
       * included in this mask.)
       * @hideinitializer
       **********************************************************************/
      ALL           = GeodesicExact::ALL,
    };

    /** \name Constructors
     **********************************************************************/
    ///@{

    /**
     * Constructor for a geodesic line staring at latitude \e lat1, longitude
     * \e lon1, and azimuth \e azi1 (all in degrees).
     *
     * @param[in] g A GeodesicExact object used to compute the necessary
     *   information about the GeodesicLineExact.
     * @param[in] lat1 latitude of point 1 (degrees).
     * @param[in] lon1 longitude of point 1 (degrees).
     * @param[in] azi1 azimuth at point 1 (degrees).
     * @param[in] caps bitor'ed combination of GeodesicLineExact::mask values
     *   specifying the capabilities the GeodesicLineExact object should
     *   possess, i.e., which quantities can be returned in calls to
     *   GeodesicLine::Position.
     *
     * \e lat1 should be in the range [&minus;90&deg;, 90&deg;].
     *
     * The GeodesicLineExact::mask values are
     * - \e caps |= GeodesicLineExact::LATITUDE for the latitude \e lat2; this
     *   is added automatically;
     * - \e caps |= GeodesicLineExact::LONGITUDE for the latitude \e lon2;
     * - \e caps |= GeodesicLineExact::AZIMUTH for the latitude \e azi2; this
     *   is added automatically;
     * - \e caps |= GeodesicLineExact::DISTANCE for the distance \e s12;
     * - \e caps |= GeodesicLineExact::REDUCEDLENGTH for the reduced length \e
         m12;
     * - \e caps |= GeodesicLineExact::GEODESICSCALE for the geodesic scales \e
     *   M12 and \e M21;
     * - \e caps |= GeodesicLineExact::AREA for the area \e S12;
     * - \e caps |= GeodesicLineExact::DISTANCE_IN permits the length of the
     *   geodesic to be given in terms of \e s12; without this capability the
     *   length can only be specified in terms of arc length;
     * - \e caps |= GeodesicLineExact::ALL for all of the above.
     * .
     * The default value of \e caps is GeodesicLineExact::ALL.
     *
     * If the point is at a pole, the azimuth is defined by keeping \e lon1
     * fixed, writing \e lat1 = &plusmn;(90&deg; &minus; &epsilon;), and taking
     * the limit &epsilon; &rarr; 0+.
     **********************************************************************/
    GeodesicLineExact(const GeodesicExact& g, real lat1, real lon1, real azi1,
                      unsigned caps = ALL);

    /**
     * A default constructor.  If GeodesicLineExact::Position is called on the
     * resulting object, it returns immediately (without doing any
     * calculations).  The object can be set with a call to
     * GeodesicExact::Line.  Use Init() to test whether object is still in this
     * uninitialized state.
     **********************************************************************/
    GeodesicLineExact() : _caps(0U) {}
    ///@}

    /** \name Position in terms of distance
     **********************************************************************/
    ///@{

    /**
     * Compute the position of point 2 which is a distance \e s12 (meters)
     * from point 1.
     *
     * @param[in] s12 distance from point 1 to point 2 (meters); it can be
     *   signed.
     * @param[out] lat2 latitude of point 2 (degrees).
     * @param[out] lon2 longitude of point 2 (degrees); requires that the
     *   GeodesicLineExact object was constructed with \e caps |=
     *   GeodesicLineExact::LONGITUDE.
     * @param[out] azi2 (forward) azimuth at point 2 (degrees).
     * @param[out] m12 reduced length of geodesic (meters); requires that the
     *   GeodesicLineExact object was constructed with \e caps |=
     *   GeodesicLineExact::REDUCEDLENGTH.
     * @param[out] M12 geodesic scale of point 2 relative to point 1
     *   (dimensionless); requires that the GeodesicLineExact object was
     *   constructed with \e caps |= GeodesicLineExact::GEODESICSCALE.
     * @param[out] M21 geodesic scale of point 1 relative to point 2
     *   (dimensionless); requires that the GeodesicLineExact object was
     *   constructed with \e caps |= GeodesicLineExact::GEODESICSCALE.
     * @param[out] S12 area under the geodesic (meters<sup>2</sup>); requires
     *   that the GeodesicLineExact object was constructed with \e caps |=
     *   GeodesicLineExact::AREA.
     * @return \e a12 arc length from point 1 to point 2 (degrees).
     *
     * The values of \e lon2 and \e azi2 returned are in the range
     * [&minus;180&deg;, 180&deg;].
     *
     * The GeodesicLineExact object \e must have been constructed with \e caps
     * |= GeodesicLineExact::DISTANCE_IN; otherwise Math::NaN() is returned and
     * no parameters are set.  Requesting a value which the GeodesicLineExact
     * object is not capable of computing is not an error; the corresponding
     * argument will not be altered.
     *
     * The following functions are overloaded versions of
     * GeodesicLineExact::Position which omit some of the output parameters.
     * Note, however, that the arc length is always computed and returned as
     * the function value.
     **********************************************************************/
    Math::real Position(real s12,
                        real& lat2, real& lon2, real& azi2,
                        real& m12, real& M12, real& M21,
                        real& S12) const {
      real t;
      return GenPosition(false, s12,
                         LATITUDE | LONGITUDE | AZIMUTH |
                         REDUCEDLENGTH | GEODESICSCALE | AREA,
                         lat2, lon2, azi2, t, m12, M12, M21, S12);
    }

    /**
     * See the documentation for GeodesicLineExact::Position.
     **********************************************************************/
    Math::real Position(real s12, real& lat2, real& lon2) const {
      real t;
      return GenPosition(false, s12,
                         LATITUDE | LONGITUDE,
                         lat2, lon2, t, t, t, t, t, t);
    }

    /**
     * See the documentation for GeodesicLineExact::Position.
     **********************************************************************/
    Math::real Position(real s12, real& lat2, real& lon2,
                        real& azi2) const {
      real t;
      return GenPosition(false, s12,
                         LATITUDE | LONGITUDE | AZIMUTH,
                         lat2, lon2, azi2, t, t, t, t, t);
    }

    /**
     * See the documentation for GeodesicLineExact::Position.
     **********************************************************************/
    Math::real Position(real s12, real& lat2, real& lon2,
                        real& azi2, real& m12) const {
      real t;
      return GenPosition(false, s12,
                         LATITUDE | LONGITUDE |
                         AZIMUTH | REDUCEDLENGTH,
                         lat2, lon2, azi2, t, m12, t, t, t);
    }

    /**
     * See the documentation for GeodesicLineExact::Position.
     **********************************************************************/
    Math::real Position(real s12, real& lat2, real& lon2,
                        real& azi2, real& M12, real& M21)
      const {
      real t;
      return GenPosition(false, s12,
                         LATITUDE | LONGITUDE |
                         AZIMUTH | GEODESICSCALE,
                         lat2, lon2, azi2, t, t, M12, M21, t);
    }

    /**
     * See the documentation for GeodesicLineExact::Position.
     **********************************************************************/
    Math::real Position(real s12,
                        real& lat2, real& lon2, real& azi2,
                        real& m12, real& M12, real& M21)
      const {
      real t;
      return GenPosition(false, s12,
                         LATITUDE | LONGITUDE | AZIMUTH |
                         REDUCEDLENGTH | GEODESICSCALE,
                         lat2, lon2, azi2, t, m12, M12, M21, t);
    }
    ///@}

    /** \name Position in terms of arc length
     **********************************************************************/
    ///@{

    /**
     * Compute the position of point 2 which is an arc length \e a12 (degrees)
     * from point 1.
     *
     * @param[in] a12 arc length from point 1 to point 2 (degrees); it can
     *   be signed.
     * @param[out] lat2 latitude of point 2 (degrees).
     * @param[out] lon2 longitude of point 2 (degrees); requires that the
     *   GeodesicLineExact object was constructed with \e caps |=
     *   GeodesicLineExact::LONGITUDE.
     * @param[out] azi2 (forward) azimuth at point 2 (degrees).
     * @param[out] s12 distance from point 1 to point 2 (meters); requires
     *   that the GeodesicLineExact object was constructed with \e caps |=
     *   GeodesicLineExact::DISTANCE.
     * @param[out] m12 reduced length of geodesic (meters); requires that the
     *   GeodesicLineExact object was constructed with \e caps |=
     *   GeodesicLineExact::REDUCEDLENGTH.
     * @param[out] M12 geodesic scale of point 2 relative to point 1
     *   (dimensionless); requires that the GeodesicLineExact object was
     *   constructed with \e caps |= GeodesicLineExact::GEODESICSCALE.
     * @param[out] M21 geodesic scale of point 1 relative to point 2
     *   (dimensionless); requires that the GeodesicLineExact object was
     *   constructed with \e caps |= GeodesicLineExact::GEODESICSCALE.
     * @param[out] S12 area under the geodesic (meters<sup>2</sup>); requires
     *   that the GeodesicLineExact object was constructed with \e caps |=
     *   GeodesicLineExact::AREA.
     *
     * The values of \e lon2 and \e azi2 returned are in the range
     * [&minus;180&deg;, 180&deg;].
     *
     * Requesting a value which the GeodesicLineExact object is not capable of
     * computing is not an error; the corresponding argument will not be
     * altered.
     *
     * The following functions are overloaded versions of
     * GeodesicLineExact::ArcPosition which omit some of the output parameters.
     **********************************************************************/
    void ArcPosition(real a12, real& lat2, real& lon2, real& azi2,
                     real& s12, real& m12, real& M12, real& M21,
                     real& S12) const {
      GenPosition(true, a12,
                  LATITUDE | LONGITUDE | AZIMUTH | DISTANCE |
                  REDUCEDLENGTH | GEODESICSCALE | AREA,
                  lat2, lon2, azi2, s12, m12, M12, M21, S12);
    }

    /**
     * See the documentation for GeodesicLineExact::ArcPosition.
     **********************************************************************/
    void ArcPosition(real a12, real& lat2, real& lon2)
      const {
      real t;
      GenPosition(true, a12,
                  LATITUDE | LONGITUDE,
                  lat2, lon2, t, t, t, t, t, t);
    }

    /**
     * See the documentation for GeodesicLineExact::ArcPosition.
     **********************************************************************/
    void ArcPosition(real a12,
                     real& lat2, real& lon2, real& azi2)
      const {
      real t;
      GenPosition(true, a12,
                  LATITUDE | LONGITUDE | AZIMUTH,
                  lat2, lon2, azi2, t, t, t, t, t);
    }

    /**
     * See the documentation for GeodesicLineExact::ArcPosition.
     **********************************************************************/
    void ArcPosition(real a12, real& lat2, real& lon2, real& azi2,
                     real& s12) const {
      real t;
      GenPosition(true, a12,
                  LATITUDE | LONGITUDE | AZIMUTH | DISTANCE,
                  lat2, lon2, azi2, s12, t, t, t, t);
    }

    /**
     * See the documentation for GeodesicLineExact::ArcPosition.
     **********************************************************************/
    void ArcPosition(real a12, real& lat2, real& lon2, real& azi2,
                     real& s12, real& m12) const {
      real t;
      GenPosition(true, a12,
                  LATITUDE | LONGITUDE | AZIMUTH |
                  DISTANCE | REDUCEDLENGTH,
                  lat2, lon2, azi2, s12, m12, t, t, t);
    }

    /**
     * See the documentation for GeodesicLineExact::ArcPosition.
     **********************************************************************/
    void ArcPosition(real a12, real& lat2, real& lon2, real& azi2,
                     real& s12, real& M12, real& M21)
      const {
      real t;
      GenPosition(true, a12,
                  LATITUDE | LONGITUDE | AZIMUTH |
                  DISTANCE | GEODESICSCALE,
                  lat2, lon2, azi2, s12, t, M12, M21, t);
    }

    /**
     * See the documentation for GeodesicLineExact::ArcPosition.
     **********************************************************************/
    void ArcPosition(real a12, real& lat2, real& lon2, real& azi2,
                     real& s12, real& m12, real& M12, real& M21)
      const {
      real t;
      GenPosition(true, a12,
                  LATITUDE | LONGITUDE | AZIMUTH |
                  DISTANCE | REDUCEDLENGTH | GEODESICSCALE,
                  lat2, lon2, azi2, s12, m12, M12, M21, t);
    }
    ///@}

    /** \name The general position function.
     **********************************************************************/
    ///@{

    /**
     * The general position function.  GeodesicLineExact::Position and
     * GeodesicLineExact::ArcPosition are defined in terms of this function.
     *
     * @param[in] arcmode boolean flag determining the meaning of the second
     *   parameter; if \e arcmode is false, then the GeodesicLineExact object
     *   must have been constructed with \e caps |=
     *   GeodesicLineExact::DISTANCE_IN.
     * @param[in] s12_a12 if \e arcmode is false, this is the distance between
     *   point 1 and point 2 (meters); otherwise it is the arc length between
     *   point 1 and point 2 (degrees); it can be signed.
     * @param[in] outmask a bitor'ed combination of GeodesicLineExact::mask
     *   values specifying which of the following parameters should be set.
     * @param[out] lat2 latitude of point 2 (degrees).
     * @param[out] lon2 longitude of point 2 (degrees); requires that the
     *   GeodesicLineExact object was constructed with \e caps |=
     *   GeodesicLineExact::LONGITUDE.
     * @param[out] azi2 (forward) azimuth at point 2 (degrees).
     * @param[out] s12 distance from point 1 to point 2 (meters); requires
     *   that the GeodesicLineExact object was constructed with \e caps |=
     *   GeodesicLineExact::DISTANCE.
     * @param[out] m12 reduced length of geodesic (meters); requires that the
     *   GeodesicLineExact object was constructed with \e caps |=
     *   GeodesicLineExact::REDUCEDLENGTH.
     * @param[out] M12 geodesic scale of point 2 relative to point 1
     *   (dimensionless); requires that the GeodesicLineExact object was
     *   constructed with \e caps |= GeodesicLineExact::GEODESICSCALE.
     * @param[out] M21 geodesic scale of point 1 relative to point 2
     *   (dimensionless); requires that the GeodesicLineExact object was
     *   constructed with \e caps |= GeodesicLineExact::GEODESICSCALE.
     * @param[out] S12 area under the geodesic (meters<sup>2</sup>); requires
     *   that the GeodesicLineExact object was constructed with \e caps |=
     *   GeodesicLineExact::AREA.
     * @return \e a12 arc length from point 1 to point 2 (degrees).
     *
     * The GeodesicLineExact::mask values possible for \e outmask are
     * - \e outmask |= GeodesicLineExact::LATITUDE for the latitude \e lat2;
     * - \e outmask |= GeodesicLineExact::LONGITUDE for the latitude \e lon2;
     * - \e outmask |= GeodesicLineExact::AZIMUTH for the latitude \e azi2;
     * - \e outmask |= GeodesicLineExact::DISTANCE for the distance \e s12;
     * - \e outmask |= GeodesicLineExact::REDUCEDLENGTH for the reduced length
     *   \e m12;
     * - \e outmask |= GeodesicLineExact::GEODESICSCALE for the geodesic scales
     *   \e M12 and \e M21;
     * - \e outmask |= GeodesicLineExact::AREA for the area \e S12;
     * - \e outmask |= GeodesicLineExact::ALL for all of the above;
     * - \e outmask |= GeodesicLineExact::LONG_UNROLL to unroll \e lon2 instead
     *   of wrapping it into the range [&minus;180&deg;, 180&deg;].
     * .
     * Requesting a value which the GeodesicLineExact object is not capable of
     * computing is not an error; the corresponding argument will not be
     * altered.  Note, however, that the arc length is always computed and
     * returned as the function value.
     *
     * With the GeodesicLineExact::LONG_UNROLL bit set, the quantity \e lon2
     * &minus; \e lon1 indicates how many times and in what sense the geodesic
     * encircles the ellipsoid.
     **********************************************************************/
    Math::real GenPosition(bool arcmode, real s12_a12, unsigned outmask,
                           real& lat2, real& lon2, real& azi2,
                           real& s12, real& m12, real& M12, real& M21,
                           real& S12) const;
    ///@}

    /** \name Setting point 3
     **********************************************************************/
    ///@{

    /**
     * Specify position of point 3 in terms of distance.
     *
     * @param[in] s13 the distance from point 1 to point 3 (meters); it
     *   can be negative.
     *
     * This is only useful if the GeodesicLineExact object has been constructed
     * with \e caps |= GeodesicLineExact::DISTANCE_IN.
     **********************************************************************/
    void SetDistance(real s13);

    /**
     * Specify position of point 3 in terms of arc length.
     *
     * @param[in] a13 the arc length from point 1 to point 3 (degrees); it
     *   can be negative.
     *
     * The distance \e s13 is only set if the GeodesicLineExact object has been
     * constructed with \e caps |= GeodesicLineExact::DISTANCE.
     **********************************************************************/
    void SetArc(real a13);

    /**
     * Specify position of point 3 in terms of either distance or arc length.
     *
     * @param[in] arcmode boolean flag determining the meaning of the second
     *   parameter; if \e arcmode is false, then the GeodesicLineExact object
     *   must have been constructed with \e caps |=
     *   GeodesicLineExact::DISTANCE_IN.
     * @param[in] s13_a13 if \e arcmode is false, this is the distance from
     *   point 1 to point 3 (meters); otherwise it is the arc length from
     *   point 1 to point 3 (degrees); it can be negative.
     **********************************************************************/
    void GenSetDistance(bool arcmode, real s13_a13);

    /** \name Inspector functions
     **********************************************************************/
    ///@{

    /**
     * @return true if the object has been initialized.
     **********************************************************************/
    bool Init() const { return _caps != 0U; }

    /**
     * @return \e lat1 the latitude of point 1 (degrees).
     **********************************************************************/
    Math::real Latitude() const
    { return Init() ? _lat1 : Math::NaN(); }

    /**
     * @return \e lon1 the longitude of point 1 (degrees).
     **********************************************************************/
    Math::real Longitude() const
    { return Init() ? _lon1 : Math::NaN(); }

    /**
     * @return \e azi1 the azimuth (degrees) of the geodesic line at point 1.
     **********************************************************************/
    Math::real Azimuth() const
    { return Init() ? _azi1 : Math::NaN(); }

    /**
     * The sine and cosine of \e azi1.
     *
     * @param[out] sazi1 the sine of \e azi1.
     * @param[out] cazi1 the cosine of \e azi1.
     **********************************************************************/
    void Azimuth(real& sazi1, real& cazi1) const
    { if (Init()) { sazi1 = _salp1; cazi1 = _calp1; } }

    /**
     * @return \e azi0 the azimuth (degrees) of the geodesic line as it crosses
     *   the equator in a northward direction.
     *
     * The result lies in [&minus;90&deg;, 90&deg;].
     **********************************************************************/
    Math::real EquatorialAzimuth() const
    { return Init() ? Math::atan2d(_salp0, _calp0) : Math::NaN(); }

    /**
     * The sine and cosine of \e azi0.
     *
     * @param[out] sazi0 the sine of \e azi0.
     * @param[out] cazi0 the cosine of \e azi0.
     **********************************************************************/
    void EquatorialAzimuth(real& sazi0, real& cazi0) const
    { if (Init()) { sazi0 = _salp0; cazi0 = _calp0; } }

    /**
     * @return \e a1 the arc length (degrees) between the northward equatorial
     *   crossing and point 1.
     *
     * The result lies in (&minus;180&deg;, 180&deg;].
     **********************************************************************/
    Math::real EquatorialArc() const {
      using std::atan2;
      return Init() ? atan2(_ssig1, _csig1) / Math::degree() : Math::NaN();
    }

    /**
     * @return \e a the equatorial radius of the ellipsoid (meters).  This is
     *   the value inherited from the GeodesicExact object used in the
     *   constructor.
     **********************************************************************/
    Math::real MajorRadius() const
    { return Init() ? _a : Math::NaN(); }

    /**
     * @return \e f the flattening of the ellipsoid.  This is the value
     *   inherited from the GeodesicExact object used in the constructor.
     **********************************************************************/
    Math::real Flattening() const
    { return Init() ? _f : Math::NaN(); }

    /**
     * @return \e caps the computational capabilities that this object was
     *   constructed with.  LATITUDE and AZIMUTH are always included.
     **********************************************************************/
    unsigned Capabilities() const { return _caps; }

    /**
     * Test what capabilities are available.
     *
     * @param[in] testcaps a set of bitor'ed GeodesicLineExact::mask values.
     * @return true if the GeodesicLineExact object has all these capabilities.
     **********************************************************************/
    bool Capabilities(unsigned testcaps) const {
      testcaps &= OUT_ALL;
      return (_caps & testcaps) == testcaps;
    }

    /**
     * The distance or arc length to point 3.
     *
     * @param[in] arcmode boolean flag determining the meaning of returned
     *   value.
     * @return \e s13 if \e arcmode is false; \e a13 if \e arcmode is true.
     **********************************************************************/
    Math::real GenDistance(bool arcmode) const
    { return Init() ? (arcmode ? _a13 : _s13) : Math::NaN(); }

    /**
     * @return \e s13, the distance to point 3 (meters).
     **********************************************************************/
    Math::real Distance() const { return GenDistance(false); }

    /**
     * @return \e a13, the arc length to point 3 (degrees).
     **********************************************************************/
    Math::real Arc() const { return GenDistance(true); }
    ///@}

  };

} // namespace GeographicLib

#endif  // GEOGRAPHICLIB_GEODESICLINEEXACT_HPP
