#pragma once
/**
 * \file NETGeographicLib/GeodesicLineExact.h
 * \brief Header for NETGeographicLib::GeodesicLineExact class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#include "NETGeographicLib.h"

namespace NETGeographicLib
{
    ref class GeodesicExact;
  /**
   * \brief .NET wrapper for GeographicLib::GeodesicLineExact.
   *
   * This class allows .NET applications to access GeographicLib::GeodesicLineExact.
   *
   * GeodesicLineExact facilitates the determination of a series of points on a
   * single geodesic.  This is a companion to the GeodesicExact class.  For
   * additional information on this class see the documentation on the
   * GeodesicLine class.
   *
   * C# Example:
   * \include example-GeodesicLineExact.cs
   * Managed C++ Example:
   * \include example-GeodesicLineExact.cpp
   * Visual Basic Example:
   * \include example-GeodesicLineExact.vb
   *
   * <B>INTERFACE DIFFERENCES:</B><BR>
   * A constructor has been provided that assumes WGS84 parameters.
   *
   * The following functions are implemented as properties:
   * Latitude, Longitude, Azimuth, EquatorialAzimuth, EquatorialArc,
   * MajorRadius, Distance, Arc, and Flattening.
   *
   * The constructors, GenPosition, and Capabilities functions accept the
   * "capabilities mask" as a NETGeographicLib::Mask rather than an
   * unsigned.  The Capabilities function returns a NETGeographicLib::Mask
   * rather than an unsigned.
   *
   * The overloaded Azimuth and EquatorialAzimuth functions that return
   * the sin and cosine terms have been renamed AzimuthSinCos and
   * EquatorialAzimuthSinCos, repectively.
   **********************************************************************/
    public ref class GeodesicLineExact
    {
    private:
        enum class captype {
          CAP_NONE = 0U,
          CAP_E    = 1U<<0,
          // Skip 1U<<1 for compatibility with Geodesic (not required)
          CAP_D    = 1U<<2,
          CAP_H    = 1U<<3,
          CAP_C4   = 1U<<4,
          CAP_ALL  = 0x1FU,
          CAP_MASK = CAP_ALL,
          OUT_ALL  = 0x7F80U,
          OUT_MASK = 0xFF80U,       // Includes LONG_UNROLL
        };
        // a pointer to the GeographicLib::GeodesicLineExact.
        GeographicLib::GeodesicLineExact* m_pGeodesicLineExact;

        // the finalizer frees the unmanaged memory when the object is destroyed.
        !GeodesicLineExact(void);
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
        enum class mask {
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
          LATITUDE      = 1U<<7  | unsigned(captype::CAP_NONE),
          /**
           * Calculate longitude \e lon2.
           * @hideinitializer
           **********************************************************************/
          LONGITUDE     = 1U<<8  | unsigned(captype::CAP_H),
          /**
           * Calculate azimuths \e azi1 and \e azi2.  (It's not necessary to
           * include this as a capability to GeodesicLineExact because this is
           * included by default.)
           * @hideinitializer
           **********************************************************************/
          AZIMUTH       = 1U<<9  | unsigned(captype::CAP_NONE),
          /**
           * Calculate distance \e s12.
           * @hideinitializer
           **********************************************************************/
          DISTANCE      = 1U<<10 | unsigned(captype::CAP_E),
          /**
           * Allow distance \e s12 to be used as input in the direct geodesic
           * problem.
           * @hideinitializer
           **********************************************************************/
          DISTANCE_IN   = 1U<<11 | unsigned(captype::CAP_E),
          /**
           * Calculate reduced length \e m12.
           * @hideinitializer
           **********************************************************************/
          REDUCEDLENGTH = 1U<<12 | unsigned(captype::CAP_D),
          /**
           * Calculate geodesic scales \e M12 and \e M21.
           * @hideinitializer
           **********************************************************************/
          GEODESICSCALE = 1U<<13 | unsigned(captype::CAP_D),
          /**
           * Calculate area \e S12.
           * @hideinitializer
           **********************************************************************/
          AREA          = 1U<<14 | unsigned(captype::CAP_C4),
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
          ALL           = unsigned(captype::OUT_ALL)| unsigned(captype::CAP_ALL),
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
         * @param[in] caps bitor'ed combination of NETGeographicLib::Mask values
         *   specifying the capabilities the GeodesicLineExact object should
         *   possess, i.e., which quantities can be returned in calls to
         *   GeodesicLine::Position.
         *
         * \e lat1 should be in the range [&minus;90&deg;, 90&deg;].
         *
         * The NETGeographicLib::Mask values are
         * - \e caps |= GeodesicLineExact::LATITUDE for the latitude \e lat2; this
         *   is added automatically;
         * - \e caps |= NETGeographicLib::Mask::LONGITUDE for the latitude \e lon2;
         * - \e caps |= NETGeographicLib::Mask::AZIMUTH for the latitude \e azi2; this is
         *   added automatically;
         * - \e caps |= NETGeographicLib::Mask::DISTANCE for the distance \e s12;
         * - \e caps |= NETGeographicLib::Mask::REDUCEDLENGTH for the reduced length \e
             m12;
         * - \e caps |= NETGeographicLib::Mask::GEODESICSCALE for the geodesic scales \e
         *   M12 and \e M21;
         * - \e caps |= NETGeographicLib::Mask::AREA for the area \e S12;
         * - \e caps |= NETGeographicLib::Mask::DISTANCE_IN permits the length of the
         *   geodesic to be given in terms of \e s12; without this capability the
         *   length can only be specified in terms of arc length;
         * - \e caps |= NETGeographicLib::Mask::ALL for all of the above.
         * .
         *
         * If the point is at a pole, the azimuth is defined by keeping \e lon1
         * fixed, writing \e lat1 = &plusmn;(90&deg; &minus; &epsilon;), and taking
         * the limit &epsilon; &rarr; 0+.
         **********************************************************************/
        GeodesicLineExact(GeodesicExact^ g, double lat1, double lon1, double azi1,
                          NETGeographicLib::Mask caps );

        /**
         * A default constructor which assumes the WGS84 ellipsoid.  See
         * constructor comments for details.
         **********************************************************************/
        GeodesicLineExact(double lat1, double lon1, double azi1,
                          NETGeographicLib::Mask caps);

        /**
        * This constructor accepts a reference to an unmanaged
        * GeodesicLineExact.
        * FOR INTERNAL USE ONLY.
        **********************************************************************/
        GeodesicLineExact(const GeographicLib::GeodesicLineExact& gle);
        ///@}

        /**
         * The destructor calls the finalizer
         **********************************************************************/
        ~GeodesicLineExact()
        { this->!GeodesicLineExact(); }

        /** \name Position in terms of distance
         **********************************************************************/
        ///@{

        /**
         * Compute the position of point 2 which is a distance \e s12 (meters)
         * from point 1.
         *
         * @param[in] s12 distance between point 1 and point 2 (meters); it can be
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
         * @return \e a12 arc length of between point 1 and point 2 (degrees).
         *
         * The values of \e lon2 and \e azi2 returned are in the range
         * [&minus;180&deg;, 180&deg;).
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
        double Position(double s12,
                    [System::Runtime::InteropServices::Out] double% lat2,
                    [System::Runtime::InteropServices::Out] double% lon2,
                    [System::Runtime::InteropServices::Out] double% azi2,
                    [System::Runtime::InteropServices::Out] double% m12,
                    [System::Runtime::InteropServices::Out] double% M12,
                    [System::Runtime::InteropServices::Out] double% M21,
                    [System::Runtime::InteropServices::Out] double% S12);

        /**
         * See the documentation for GeodesicLineExact::Position.
         **********************************************************************/
        double Position(double s12,
            [System::Runtime::InteropServices::Out] double% lat2,
            [System::Runtime::InteropServices::Out] double% lon2);

        /**
         * See the documentation for GeodesicLineExact::Position.
         **********************************************************************/
        double Position(double s12,
            [System::Runtime::InteropServices::Out] double% lat2,
            [System::Runtime::InteropServices::Out] double% lon2,
            [System::Runtime::InteropServices::Out] double% azi2);

        /**
         * See the documentation for GeodesicLineExact::Position.
         **********************************************************************/
        double Position(double s12,
            [System::Runtime::InteropServices::Out] double% lat2,
            [System::Runtime::InteropServices::Out] double% lon2,
            [System::Runtime::InteropServices::Out] double% azi2,
            [System::Runtime::InteropServices::Out] double% m12);

        /**
         * See the documentation for GeodesicLineExact::Position.
         **********************************************************************/
        double Position(double s12,
            [System::Runtime::InteropServices::Out] double% lat2,
            [System::Runtime::InteropServices::Out] double% lon2,
            [System::Runtime::InteropServices::Out] double% azi2,
            [System::Runtime::InteropServices::Out] double% M12,
            [System::Runtime::InteropServices::Out] double% M21);

        /**
         * See the documentation for GeodesicLineExact::Position.
         **********************************************************************/
        double Position(double s12,
            [System::Runtime::InteropServices::Out] double% lat2,
            [System::Runtime::InteropServices::Out] double% lon2,
            [System::Runtime::InteropServices::Out] double% azi2,
            [System::Runtime::InteropServices::Out] double% m12,
            [System::Runtime::InteropServices::Out] double% M12,
            [System::Runtime::InteropServices::Out] double% M21);

        ///@}

        /** \name Position in terms of arc length
         **********************************************************************/
        ///@{

        /**
         * Compute the position of point 2 which is an arc length \e a12 (degrees)
         * from point 1.
         *
         * @param[in] a12 arc length between point 1 and point 2 (degrees); it can
         *   be signed.
         * @param[out] lat2 latitude of point 2 (degrees).
         * @param[out] lon2 longitude of point 2 (degrees); requires that the
         *   GeodesicLineExact object was constructed with \e caps |=
         *   GeodesicLineExact::LONGITUDE.
         * @param[out] azi2 (forward) azimuth at point 2 (degrees).
         * @param[out] s12 distance between point 1 and point 2 (meters); requires
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
         * [&minus;180&deg;, 180&deg;).
         *
         * Requesting a value which the GeodesicLineExact object is not capable of
         * computing is not an error; the corresponding argument will not be
         * altered.
         *
         * The following functions are overloaded versions of
         * GeodesicLineExact::ArcPosition which omit some of the output parameters.
         **********************************************************************/
        void ArcPosition(double a12,
            [System::Runtime::InteropServices::Out] double% lat2,
            [System::Runtime::InteropServices::Out] double% lon2,
            [System::Runtime::InteropServices::Out] double% azi2,
            [System::Runtime::InteropServices::Out] double% s12,
            [System::Runtime::InteropServices::Out] double% m12,
            [System::Runtime::InteropServices::Out] double% M12,
            [System::Runtime::InteropServices::Out] double% M21,
            [System::Runtime::InteropServices::Out] double% S12);

        /**
         * See the documentation for GeodesicLineExact::ArcPosition.
         **********************************************************************/
        void ArcPosition(double a12,
            [System::Runtime::InteropServices::Out] double% lat2,
            [System::Runtime::InteropServices::Out] double% lon2);

        /**
         * See the documentation for GeodesicLineExact::ArcPosition.
         **********************************************************************/
        void ArcPosition(double a12,
            [System::Runtime::InteropServices::Out] double% lat2,
            [System::Runtime::InteropServices::Out] double% lon2,
            [System::Runtime::InteropServices::Out] double% azi2);

        /**
         * See the documentation for GeodesicLineExact::ArcPosition.
         **********************************************************************/
        void ArcPosition(double a12,
            [System::Runtime::InteropServices::Out] double% lat2,
            [System::Runtime::InteropServices::Out] double% lon2,
            [System::Runtime::InteropServices::Out] double% azi2,
            [System::Runtime::InteropServices::Out] double% s12);

        /**
         * See the documentation for GeodesicLineExact::ArcPosition.
         **********************************************************************/
        void ArcPosition(double a12,
            [System::Runtime::InteropServices::Out] double% lat2,
            [System::Runtime::InteropServices::Out] double% lon2,
            [System::Runtime::InteropServices::Out] double% azi2,
            [System::Runtime::InteropServices::Out] double% s12,
            [System::Runtime::InteropServices::Out] double% m12);

        /**
         * See the documentation for GeodesicLineExact::ArcPosition.
         **********************************************************************/
        void ArcPosition(double a12,
            [System::Runtime::InteropServices::Out] double% lat2,
            [System::Runtime::InteropServices::Out] double% lon2,
            [System::Runtime::InteropServices::Out] double% azi2,
            [System::Runtime::InteropServices::Out] double% s12,
            [System::Runtime::InteropServices::Out] double% M12,
            [System::Runtime::InteropServices::Out] double% M21);

        /**
         * See the documentation for GeodesicLineExact::ArcPosition.
         **********************************************************************/
        void ArcPosition(double a12,
            [System::Runtime::InteropServices::Out] double% lat2,
            [System::Runtime::InteropServices::Out] double% lon2,
            [System::Runtime::InteropServices::Out] double% azi2,
            [System::Runtime::InteropServices::Out] double% s12,
            [System::Runtime::InteropServices::Out] double% m12,
            [System::Runtime::InteropServices::Out] double% M12,
            [System::Runtime::InteropServices::Out] double% M21);
        ///@}

        /** \name The general position function.
         **********************************************************************/
        ///@{

        /**
         * The general position function.  GeodesicLineExact::Position and
         * GeodesicLineExact::ArcPosition are defined in terms of this function.
         *
         * @param[in] arcmode boolean flag determining the meaning of the second
         *   parameter; if arcmode is false, then the GeodesicLineExact object must
         *   have been constructed with \e caps |= GeodesicLineExact::DISTANCE_IN.
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
         * @param[out] s12 distance between point 1 and point 2 (meters); requires
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
         * @return \e a12 arc length of between point 1 and point 2 (degrees).
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
         * - \e outmask |= GeodesicLineExact::LONG_UNROLL  to unroll \e lon2 instead
         *   of wrapping it into the range [&minus;180&deg;, 180&deg;).
         * .
         * Requesting a value which the GeodesicLineExact object is not capable of
         * computing is not an error; the corresponding argument will not be
         * altered.  Note, however, that the arc length is always computed and
         * returned as the function value.
         *
         * With the LONG_UNROLL bit set, the quantity \e lon2 &minus; \e lon1
         * indicates how many times and in what sense the geodesic encircles
         * the ellipsoid.
         **********************************************************************/
        double GenPosition(bool arcmode, double s12_a12,
                GeodesicLineExact::mask outmask,
                [System::Runtime::InteropServices::Out] double% lat2,
                [System::Runtime::InteropServices::Out] double% lon2,
                [System::Runtime::InteropServices::Out] double% azi2,
                [System::Runtime::InteropServices::Out] double% s12,
                [System::Runtime::InteropServices::Out] double% m12,
                [System::Runtime::InteropServices::Out] double% M12,
                [System::Runtime::InteropServices::Out] double% M21,
                [System::Runtime::InteropServices::Out] double% S12);

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
        void SetDistance(double s13);

        /**
        * Specify position of point 3 in terms of arc length.
        *
        * @param[in] a13 the arc length from point 1 to point 3 (degrees); it
        *   can be negative.
        *
        * The distance \e s13 is only set if the GeodesicLineExact object has been
        * constructed with \e caps |= GeodesicLineExact::DISTANCE.
        **********************************************************************/
        void SetArc(double a13);

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
        void GenSetDistance(bool arcmode, double s13_a13);

        /**
        * The distance or arc length to point 3.
        *
        * @param[in] arcmode boolean flag determining the meaning of returned
        *   value.
        * @return \e s13 if \e arcmode is false; \e a13 if \e arcmode is true.
        **********************************************************************/
        double GenDistance(bool arcmode);
        ///@}

        /** \name Trigonometric accessor functions
        **********************************************************************/
        ///@{
        /**
        * The sine and cosine of \e azi1.
        *
        * @param[out] sazi1 the sine of \e azi1.
        * @param[out] cazi1 the cosine of \e azi1.
        **********************************************************************/
        void AzimuthSinCos(
            [System::Runtime::InteropServices::Out] double% sazi1,
            [System::Runtime::InteropServices::Out] double% cazi1);

        /**
        * The sine and cosine of \e azi0.
        *
        * @param[out] sazi0 the sine of \e azi0.
        * @param[out] cazi0 the cosine of \e azi0.
        **********************************************************************/
        void EquatorialAzimuthSinCos(
            [System::Runtime::InteropServices::Out] double% sazi0,
            [System::Runtime::InteropServices::Out] double% cazi0);
        ///@}

        /** \name Inspector functions
         **********************************************************************/
        ///@{
        /**
         * @return \e lat1 the latitude of point 1 (degrees).
         **********************************************************************/
        property double Latitude { double get(); }

        /**
         * @return \e lon1 the longitude of point 1 (degrees).
         **********************************************************************/
        property double Longitude { double get(); }

        /**
         * @return \e azi1 the azimuth (degrees) of the geodesic line at point 1.
         **********************************************************************/
        property double Azimuth { double get(); }

        /**
         * @return \e azi0 the azimuth (degrees) of the geodesic line as it crosses
         * the equator in a northward direction.
         **********************************************************************/
        property double EquatorialAzimuth { double get(); }

        /**
         * @return \e a1 the arc length (degrees) between the northward equatorial
         * crossing and point 1.
         **********************************************************************/
        property double EquatorialArc { double get(); }

        /**
         * @return \e a the equatorial radius of the ellipsoid (meters).  This is
         *   the value inherited from the GeodesicExact object used in the
         *   constructor.
         **********************************************************************/
        property double MajorRadius { double get(); }

        /**
         * @return \e f the flattening of the ellipsoid.  This is the value
         *   inherited from the GeodesicExact object used in the constructor.
         **********************************************************************/
        property double Flattening { double get(); }

        /**
        * @return \e s13, the distance to point 3 (meters).
        **********************************************************************/
        property double Distance { double get(); }

        /**
        * @return \e a13, the arc length to point 3 (degrees).
        **********************************************************************/
        property double Arc { double get(); }

        /**
         * @return \e caps the computational capabilities that this object was
         *   constructed with.  LATITUDE and AZIMUTH are always included.
         **********************************************************************/
        NETGeographicLib::Mask Capabilities();

        /**
         * @param[in] testcaps a set of bitor'ed GeodesicLineExact::mask values.
         * @return true if the GeodesicLineExact object has all these capabilities.
         **********************************************************************/
        bool Capabilities(NETGeographicLib::Mask testcaps);
        ///@}
    };
} // namespace NETGeographicLib
