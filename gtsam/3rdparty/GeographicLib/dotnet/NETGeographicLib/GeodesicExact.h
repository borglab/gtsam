#pragma once
/**
 * \file NETGeographicLib/GeodesicExact.h
 * \brief Header for NETGeographicLib::GeodesicExact class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * http://geographiclib.sourceforge.net/
 **********************************************************************/
#include "NETGeographicLib.h"

namespace NETGeographicLib
{
    ref class GeodesicLineExact;
    /*!
    \brief .NET wrapper for GeographicLib::GeodesicExact.

    This class allows .NET applications to access GeographicLib::GeodesicExact.
    */
  /**
   * \brief .NET wrapper for GeographicLib::GeodesicExact.
   *
   * This class allows .NET applications to access GeographicLib::GeodesicExact.
   *
   * The equations for geodesics on an ellipsoid can be expressed in terms of
   * incomplete elliptic integrals.  The Geodesic class expands these integrals
   * in a series in the flattening \e f and this provides an accurate solution
   * for \e f &isin [-0.01, 0.01].  The GeodesicExact class computes the
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
   * C# Example:
   * \include example-GeodesicExact.cs
   * Managed C++ Example:
   * \include example-GeodesicExact.cpp
   * Visual Basic Example:
   * \include example-GeodesicExact.vb
   *
   * <B>INTERFACE DIFFERENCES:</B><BR>
   * A default constructor is provided that assumes WGS84 parameters.
   *
   * The MajorRadius, Flattening, and EllipsoidArea functions are
   * implemented as properties.
   *
   * The GenDirect, GenInverse, and Line functions accept the
   * "capabilities mask" as a NETGeographicLib::Mask rather than an
   * unsigned.
   **********************************************************************/
    public ref class GeodesicExact
    {
        private:
        // pointer to the unmanaged GeographicLib::GeodesicExact.
        const GeographicLib::GeodesicExact* m_pGeodesicExact;

        // the finalizer deletes the unmanaged memory.
        !GeodesicExact();
    public:
        /** \name Constructor
         **********************************************************************/
        ///@{
        /**
         * Constructor for a WGS84 ellipsoid
         **********************************************************************/
        GeodesicExact();

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
        GeodesicExact(double a, double f);
        ///@}

        /**
         * The desstructor calls the finalizer.
         **********************************************************************/
        ~GeodesicExact()
        { this->!GeodesicExact(); }

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
        double Direct(double lat1, double lon1, double azi1, double s12,
                          [System::Runtime::InteropServices::Out] double% lat2,
                          [System::Runtime::InteropServices::Out] double% lon2,
                          [System::Runtime::InteropServices::Out] double% azi2,
                          [System::Runtime::InteropServices::Out] double% m12,
                          [System::Runtime::InteropServices::Out] double% M12,
                          [System::Runtime::InteropServices::Out] double% M21,
                          [System::Runtime::InteropServices::Out] double% S12);

        /**
         * See the documentation for GeodesicExact::Direct.
         **********************************************************************/
        double Direct(double lat1, double lon1, double azi1, double s12,
                          [System::Runtime::InteropServices::Out] double% lat2,
                          [System::Runtime::InteropServices::Out] double% lon2);

        /**
         * See the documentation for GeodesicExact::Direct.
         **********************************************************************/
        double Direct(double lat1, double lon1, double azi1, double s12,
                          [System::Runtime::InteropServices::Out] double% lat2,
                          [System::Runtime::InteropServices::Out] double% lon2,
                          [System::Runtime::InteropServices::Out] double% azi2);

        /**
         * See the documentation for GeodesicExact::Direct.
         **********************************************************************/
        double Direct(double lat1, double lon1, double azi1, double s12,
                          [System::Runtime::InteropServices::Out] double% lat2,
                          [System::Runtime::InteropServices::Out] double% lon2,
                          [System::Runtime::InteropServices::Out] double% azi2,
                          [System::Runtime::InteropServices::Out] double% m12);

        /**
         * See the documentation for GeodesicExact::Direct.
         **********************************************************************/
        double Direct(double lat1, double lon1, double azi1, double s12,
                          [System::Runtime::InteropServices::Out] double% lat2,
                          [System::Runtime::InteropServices::Out] double% lon2,
                          [System::Runtime::InteropServices::Out] double% azi2,
                          [System::Runtime::InteropServices::Out] double% M12,
                          [System::Runtime::InteropServices::Out] double% M21);

        /**
         * See the documentation for GeodesicExact::Direct.
         **********************************************************************/
        double Direct(double lat1, double lon1, double azi1, double s12,
                          [System::Runtime::InteropServices::Out] double% lat2,
                          [System::Runtime::InteropServices::Out] double% lon2,
                          [System::Runtime::InteropServices::Out] double% azi2,
                          [System::Runtime::InteropServices::Out] double% m12,
                          [System::Runtime::InteropServices::Out] double% M12,
                          [System::Runtime::InteropServices::Out] double% M21);
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
        void ArcDirect(double lat1, double lon1, double azi1, double a12,
                       [System::Runtime::InteropServices::Out] double% lat2,
                       [System::Runtime::InteropServices::Out] double% lon2,
                       [System::Runtime::InteropServices::Out] double% azi2,
                       [System::Runtime::InteropServices::Out] double% s12,
                       [System::Runtime::InteropServices::Out] double% m12,
                       [System::Runtime::InteropServices::Out] double% M12,
                       [System::Runtime::InteropServices::Out] double% M21,
                       [System::Runtime::InteropServices::Out] double% S12);

        /**
         * See the documentation for GeodesicExact::ArcDirect.
         **********************************************************************/
        void ArcDirect(double lat1, double lon1, double azi1, double a12,
                       [System::Runtime::InteropServices::Out] double% lat2,
                       [System::Runtime::InteropServices::Out] double% lon2);

        /**
         * See the documentation for GeodesicExact::ArcDirect.
         **********************************************************************/
        void ArcDirect(double lat1, double lon1, double azi1, double a12,
                       [System::Runtime::InteropServices::Out] double% lat2,
                       [System::Runtime::InteropServices::Out] double% lon2,
                       [System::Runtime::InteropServices::Out] double% azi2);

        /**
         * See the documentation for GeodesicExact::ArcDirect.
         **********************************************************************/
        void ArcDirect(double lat1, double lon1, double azi1, double a12,
                       [System::Runtime::InteropServices::Out] double% lat2,
                       [System::Runtime::InteropServices::Out] double% lon2,
                       [System::Runtime::InteropServices::Out] double% azi2,
                       [System::Runtime::InteropServices::Out] double% s12);

        /**
         * See the documentation for GeodesicExact::ArcDirect.
         **********************************************************************/
        void ArcDirect(double lat1, double lon1, double azi1, double a12,
                       [System::Runtime::InteropServices::Out] double% lat2,
                       [System::Runtime::InteropServices::Out] double% lon2,
                       [System::Runtime::InteropServices::Out] double% azi2,
                       [System::Runtime::InteropServices::Out] double% s12,
                       [System::Runtime::InteropServices::Out] double% m12);

        /**
         * See the documentation for GeodesicExact::ArcDirect.
         **********************************************************************/
        void ArcDirect(double lat1, double lon1, double azi1, double a12,
                       [System::Runtime::InteropServices::Out] double% lat2,
                       [System::Runtime::InteropServices::Out] double% lon2,
                       [System::Runtime::InteropServices::Out] double% azi2,
                       [System::Runtime::InteropServices::Out] double% s12,
                       [System::Runtime::InteropServices::Out] double% M12,
                       [System::Runtime::InteropServices::Out] double% M21);

        /**
         * See the documentation for GeodesicExact::ArcDirect.
         **********************************************************************/
        void ArcDirect(double lat1, double lon1, double azi1, double a12,
                       [System::Runtime::InteropServices::Out] double% lat2,
                       [System::Runtime::InteropServices::Out] double% lon2,
                       [System::Runtime::InteropServices::Out] double% azi2,
                       [System::Runtime::InteropServices::Out] double% s12,
                       [System::Runtime::InteropServices::Out] double% m12,
                       [System::Runtime::InteropServices::Out] double% M12,
                       [System::Runtime::InteropServices::Out] double% M21);
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
         * @param[in] outmask a bitor'ed combination of  NETGeographicLib::Mask values
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
         * The  NETGeographicLib::Mask values possible for \e outmask are
         * - \e outmask |= NETGeographicLib::Mask::LATITUDE for the latitude \e lat2;
         * - \e outmask |= NETGeographicLib::Mask::LONGITUDE for the latitude \e lon2;
         * - \e outmask |= NETGeographicLib::Mask::AZIMUTH for the latitude \e azi2;
         * - \e outmask |= NETGeographicLib::Mask::DISTANCE for the distance \e s12;
         * - \e outmask |= NETGeographicLib::Mask::REDUCEDLENGTH for the reduced length \e
         *   m12;
         * - \e outmask |= NETGeographicLib::Mask::GEODESICSCALE for the geodesic scales \e
         *   M12 and \e M21;
         * - \e outmask |= NETGeographicLib::Mask::AREA for the area \e S12;
         * - \e outmask |= NETGeographicLib::Mask::ALL for all of the above.
         * .
         * The function value \e a12 is always computed and returned and this
         * equals \e s12_a12 is \e arcmode is true.  If \e outmask includes
         * GeodesicExact::DISTANCE and \e arcmode is false, then \e s12 = \e
         * s12_a12.  It is not necessary to include  NETGeographicLib::Mask::DISTANCE_IN in
         * \e outmask; this is automatically included is \e arcmode is false.
         **********************************************************************/
        double GenDirect(double lat1, double lon1, double azi1,
                        bool arcmode, double s12_a12, NETGeographicLib::Mask outmask,
                        [System::Runtime::InteropServices::Out] double% lat2,
                        [System::Runtime::InteropServices::Out] double% lon2,
                        [System::Runtime::InteropServices::Out] double% azi2,
                        [System::Runtime::InteropServices::Out] double% s12,
                        [System::Runtime::InteropServices::Out] double% m12,
                        [System::Runtime::InteropServices::Out] double% M12,
                        [System::Runtime::InteropServices::Out] double% M21,
                        [System::Runtime::InteropServices::Out] double% S12);
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
        double Inverse(double lat1, double lon1, double lat2, double lon2,
                           [System::Runtime::InteropServices::Out] double% s12,
                           [System::Runtime::InteropServices::Out] double% azi1,
                           [System::Runtime::InteropServices::Out] double% azi2,
                           [System::Runtime::InteropServices::Out] double% m12,
                           [System::Runtime::InteropServices::Out] double% M12,
                           [System::Runtime::InteropServices::Out] double% M21,
                           [System::Runtime::InteropServices::Out] double% S12);

        /**
         * See the documentation for GeodesicExact::Inverse.
         **********************************************************************/
        double Inverse(double lat1, double lon1, double lat2, double lon2,
                           [System::Runtime::InteropServices::Out] double% s12);

        /**
         * See the documentation for GeodesicExact::Inverse.
         **********************************************************************/
        double Inverse(double lat1, double lon1, double lat2, double lon2,
                           [System::Runtime::InteropServices::Out] double% azi1,
                           [System::Runtime::InteropServices::Out] double% azi2);

        /**
         * See the documentation for GeodesicExact::Inverse.
         **********************************************************************/
        double Inverse(double lat1, double lon1, double lat2, double lon2,
                    [System::Runtime::InteropServices::Out] double% s12,
                    [System::Runtime::InteropServices::Out] double% azi1,
                    [System::Runtime::InteropServices::Out] double% azi2);

        /**
         * See the documentation for GeodesicExact::Inverse.
         **********************************************************************/
        double Inverse(double lat1, double lon1, double lat2, double lon2,
                    [System::Runtime::InteropServices::Out] double% s12,
                    [System::Runtime::InteropServices::Out] double% azi1,
                    [System::Runtime::InteropServices::Out] double% azi2,
                    [System::Runtime::InteropServices::Out] double% m12);

        /**
         * See the documentation for GeodesicExact::Inverse.
         **********************************************************************/
        double Inverse(double lat1, double lon1, double lat2, double lon2,
                    [System::Runtime::InteropServices::Out] double% s12,
                    [System::Runtime::InteropServices::Out] double% azi1,
                    [System::Runtime::InteropServices::Out] double% azi2,
                    [System::Runtime::InteropServices::Out] double% M12,
                    [System::Runtime::InteropServices::Out] double% M21);

        /**
         * See the documentation for GeodesicExact::Inverse.
         **********************************************************************/
        double Inverse(double lat1, double lon1, double lat2, double lon2,
                    [System::Runtime::InteropServices::Out] double% s12,
                    [System::Runtime::InteropServices::Out] double% azi1,
                    [System::Runtime::InteropServices::Out] double% azi2,
                    [System::Runtime::InteropServices::Out] double% m12,
                    [System::Runtime::InteropServices::Out] double% M12,
                    [System::Runtime::InteropServices::Out] double% M21);
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
         * @param[in] outmask a bitor'ed combination of  NETGeographicLib::Mask values
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
         * The NETGeographicLib::Mask values possible for \e outmask are
         * - \e outmask |= NETGeographicLib::Mask::DISTANCE for the distance \e s12;
         * - \e outmask |= NETGeographicLib::Mask::AZIMUTH for the latitude \e azi2;
         * - \e outmask |= NETGeographicLib::Mask::REDUCEDLENGTH for the reduced length \e
         *   m12;
         * - \e outmask |= NETGeographicLib::Mask::GEODESICSCALE for the geodesic scales \e
         *   M12 and \e M21;
         * - \e outmask |= NETGeographicLib::Mask::AREA for the area \e S12;
         * - \e outmask |= NETGeographicLib::Mask::ALL for all of the above.
         * .
         * The arc length is always computed and returned as the function value.
         **********************************************************************/
        double GenInverse(double lat1, double lon1, double lat2, double lon2,
                        NETGeographicLib::Mask outmask,
                        [System::Runtime::InteropServices::Out] double% s12,
                        [System::Runtime::InteropServices::Out] double% azi1,
                        [System::Runtime::InteropServices::Out] double% azi2,
                        [System::Runtime::InteropServices::Out] double% m12,
                        [System::Runtime::InteropServices::Out] double% M12,
                        [System::Runtime::InteropServices::Out] double% M21,
                        [System::Runtime::InteropServices::Out] double% S12);
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
         * @param[in] caps bitor'ed combination of NETGeographicLib::Mask values
         *   specifying the capabilities the GeodesicLineExact object should
         *   possess, i.e., which quantities can be returned in calls to
         *   GeodesicLineExact::Position.
         * @return a GeodesicLineExact object.
         *
         * \e lat1 should be in the range [&minus;90&deg;, 90&deg;]; \e lon1 and \e
         * azi1 should be in the range [&minus;540&deg;, 540&deg;).
         *
         * The GeodesicExact::mask values are
         * - \e caps |= NETGeographicLib::Mask::LATITUDE for the latitude \e lat2; this is
         *   added automatically;
         * - \e caps |= NETGeographicLib::Mask::LONGITUDE for the latitude \e lon2;
         * - \e caps |= NETGeographicLib::Mask::AZIMUTH for the azimuth \e azi2; this is
         *   added automatically;
         * - \e caps |= NETGeographicLib::Mask::DISTANCE for the distance \e s12;
         * - \e caps |= NETGeographicLib::Mask::REDUCEDLENGTH for the reduced length \e m12;
         * - \e caps |= NETGeographicLib::Mask::GEODESICSCALE for the geodesic scales \e M12
         *   and \e M21;
         * - \e caps |= NETGeographicLib::Mask::AREA for the area \e S12;
         * - \e caps |= NETGeographicLib::Mask::DISTANCE_IN permits the length of the
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
        GeodesicLineExact^ Line(double lat1, double lon1, double azi1,
            NETGeographicLib::Mask caps );

        ///@}

        /** \name Inspector functions.
         **********************************************************************/
        ///@{

        /**
         * @return \e a the equatorial radius of the ellipsoid (meters).  This is
         *   the value used in the constructor.
         **********************************************************************/
        property double MajorRadius { double get(); }

        /**
         * @return \e f the  flattening of the ellipsoid.  This is the
         *   value used in the constructor.
         **********************************************************************/
        property double Flattening { double get(); }

        /**
         * @return total area of ellipsoid in meters<sup>2</sup>.  The area of a
         *   polygon encircling a pole can be found by adding
         *   GeodesicExact::EllipsoidArea()/2 to the sum of \e S12 for each side of
         *   the polygon.
         **********************************************************************/
        property double EllipsoidArea { double get(); }
        ///@}

        /**
         * @return A pointer to the unmanaged GeographicLib::GeodesicExact.
         *
         * This function is for internal use only.
         **********************************************************************/
        System::IntPtr^ GetUnmanaged();
    };
} // namespace NETGeographicLib
