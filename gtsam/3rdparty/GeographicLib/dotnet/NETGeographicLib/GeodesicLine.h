#pragma once
/**
 * \file NETGeographicLib/GeodesicLine.h
 * \brief Header for NETGeographicLib::GeodesicLine class
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
  /**
   * \brief .NET wrapper for GeographicLib::GeodesicLine.
   *
   * This class allows .NET applications to access GeographicLib::GeodesicLine.
   *
   * GeodesicLine facilitates the determination of a series of points on a
   * single geodesic.  The starting point (\e lat1, \e lon1) and the azimuth \e
   * azi1 are specified in the constructor.  GeodesicLine.Position returns the
   * location of point 2 a distance \e s12 along the geodesic.  Alternatively
   * GeodesicLine.ArcPosition gives the position of point 2 an arc length \e
   * a12 along the geodesic.
   *
   * The default copy constructor and assignment operators work with this
   * class.  Similarly, a vector can be used to hold GeodesicLine objects.
   *
   * The calculations are accurate to better than 15 nm (15 nanometers).  See
   * Sec. 9 of
   * <a href="http://arxiv.org/abs/1102.1215v1">arXiv:1102.1215v1</a> for
   * details.  The algorithms used by this class are based on series expansions
   * using the flattening \e f as a small parameter.  These are only accurate
   * for |<i>f</i>| &lt; 0.02; however reasonably accurate results will be
   * obtained for |<i>f</i>| &lt; 0.2.  For very eccentric ellipsoids, use
   * GeodesicLineExact instead.
   *
   * The algorithms are described in
   * - C. F. F. Karney,
   *   <a href="http://dx.doi.org/10.1007/s00190-012-0578-z">
   *   Algorithms for geodesics</a>,
   *   J. Geodesy <b>87</b>, 43--55 (2013);
   *   DOI: <a href="http://dx.doi.org/10.1007/s00190-012-0578-z">
   *   10.1007/s00190-012-0578-z</a>;
   *   addenda: <a href="http://geographiclib.sf.net/geod-addenda.html">
   *   geod-addenda.html</a>.
   * .
   * For more information on geodesics see \ref geodesic.
   *
   * C# Example:
   * \include example-GeodesicLine.cs
   * Managed C++ Example:
   * \include example-GeodesicLine.cpp
   * Visual Basic Example:
   * \include example-GeodesicLine.vb
   *
   * <B>INTERFACE DIFFERENCES:</B><BR>
   * A constructor has been provided which assumes WGS84 parameters.
   *
   * The following functions are implemented as properties:
   * Latitude, Longitude, Azimuth, EquatorialAzimuth, EquatorialArc,
   * MajorRadius, and Flattening.
   *
   * The constructors, Capabilities, and GenPosition functions accept the
   * "capabilities mask" as a NETGeographicLib::Mask rather than an
   * unsigned.  The Capabilities function returns a  NETGeographicLib::Mask
   * rather than an unsigned.
   **********************************************************************/
    public ref class GeodesicLine
    {
        private:
        // pointer to the unmanaged GeographicLib::GeodesicLine.
        const GeographicLib::GeodesicLine* m_pGeodesicLine;

        // The finalizer frees the unmanaged memory when this object is destroyed.
        !GeodesicLine(void);
    public:
        /** \name Constructors
         **********************************************************************/
        ///@{

        /**
         * Constructor for a geodesic line staring at latitude \e lat1, longitude
         * \e lon1, and azimuth \e azi1 (all in degrees).
         *
         * @param[in] g A Geodesic object used to compute the necessary information
         *   about the GeodesicLine.
         * @param[in] lat1 latitude of point 1 (degrees).
         * @param[in] lon1 longitude of point 1 (degrees).
         * @param[in] azi1 azimuth at point 1 (degrees).
         * @param[in] caps bitor'ed combination of NETGeographicLib::Mask values
         *   specifying the capabilities the GeodesicLine object should possess,
         *   i.e., which quantities can be returned in calls to
         *   GeodesicLine::Position.
         *
         * \e lat1 should be in the range [&minus;90&deg;, 90&deg;]; \e lon1 and \e
         * azi1 should be in the range [&minus;540&deg;, 540&deg;).
         *
         * The NETGeographicLib::Mask values are
         * - \e caps |= GeodesicLine::LATITUDE for the latitude \e lat2; this is
         *   added automatically;
         * - \e caps |= GeodesicLine::LONGITUDE for the latitude \e lon2;
         * - \e caps |= GeodesicLine::AZIMUTH for the latitude \e azi2; this is
         *   added automatically;
         * - \e caps |= GeodesicLine::DISTANCE for the distance \e s12;
         * - \e caps |= GeodesicLine::REDUCEDLENGTH for the reduced length \e m12;
         * - \e caps |= GeodesicLine::GEODESICSCALE for the geodesic scales \e M12
         *   and \e M21;
         * - \e caps |= GeodesicLine::AREA for the area \e S12;
         * - \e caps |= GeodesicLine::DISTANCE_IN permits the length of the
         *   geodesic to be given in terms of \e s12; without this capability the
         *   length can only be specified in terms of arc length;
         * - \e caps |= GeodesicLine::ALL for all of the above.
         * .
         * The default value of \e caps is GeodesicLine::ALL.
         *
         * If the point is at a pole, the azimuth is defined by keeping \e lon1
         * fixed, writing \e lat1 = &plusmn;(90&deg; &minus; &epsilon;), and taking
         * the limit &epsilon; &rarr; 0+.
         **********************************************************************/
        GeodesicLine( Geodesic^ g, double lat1, double lon1, double azi1,
                      NETGeographicLib::Mask caps );

        /**
         * A constructor which assumes the WGS84 ellipsoid.
         **********************************************************************/
        GeodesicLine(double lat1, double lon1, double azi1,
                     NETGeographicLib::Mask caps);
        ///@}

        /**
         * The destructor calls the finalizer.
         **********************************************************************/
        ~GeodesicLine()
        { this->!GeodesicLine(); }

        /** \name Position in terms of distance
         **********************************************************************/
        ///@{

        /**
         * Compute the position of point 2 which is a distance \e s12 (meters) from
         * point 1.
         *
         * @param[in] s12 distance between point 1 and point 2 (meters); it can be
         *   negative.
         * @param[out] lat2 latitude of point 2 (degrees).
         * @param[out] lon2 longitude of point 2 (degrees); requires that the
         *   GeodesicLine object was constructed with \e caps |=
         *   GeodesicLine::LONGITUDE.
         * @param[out] azi2 (forward) azimuth at point 2 (degrees).
         * @param[out] m12 reduced length of geodesic (meters); requires that the
         *   GeodesicLine object was constructed with \e caps |=
         *   GeodesicLine::REDUCEDLENGTH.
         * @param[out] M12 geodesic scale of point 2 relative to point 1
         *   (dimensionless); requires that the GeodesicLine object was constructed
         *   with \e caps |= GeodesicLine::GEODESICSCALE.
         * @param[out] M21 geodesic scale of point 1 relative to point 2
         *   (dimensionless); requires that the GeodesicLine object was constructed
         *   with \e caps |= GeodesicLine::GEODESICSCALE.
         * @param[out] S12 area under the geodesic (meters<sup>2</sup>); requires
         *   that the GeodesicLine object was constructed with \e caps |=
         *   GeodesicLine::AREA.
         * @return \e a12 arc length of between point 1 and point 2 (degrees).
         *
         * The values of \e lon2 and \e azi2 returned are in the range
         * [&minus;180&deg;, 180&deg;).
         *
         * The GeodesicLine object \e must have been constructed with \e caps |=
         * GeodesicLine::DISTANCE_IN; otherwise Math::NaN() is returned and no
         * parameters are set.  Requesting a value which the GeodesicLine object is
         * not capable of computing is not an error; the corresponding argument
         * will not be altered.
         *
         * The following functions are overloaded versions of
         * GeodesicLine::Position which omit some of the output parameters.  Note,
         * however, that the arc length is always computed and returned as the
         * function value.
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
         * See the documentation for GeodesicLine::Position.
         **********************************************************************/
        double Position(double s12,
            [System::Runtime::InteropServices::Out] double% lat2,
            [System::Runtime::InteropServices::Out] double% lon2);

        /**
         * See the documentation for GeodesicLine::Position.
         **********************************************************************/
        double Position(double s12,
            [System::Runtime::InteropServices::Out] double% lat2,
            [System::Runtime::InteropServices::Out] double% lon2,
            [System::Runtime::InteropServices::Out] double% azi2);

        /**
         * See the documentation for GeodesicLine::Position.
         **********************************************************************/
        double Position(double s12,
            [System::Runtime::InteropServices::Out] double% lat2,
            [System::Runtime::InteropServices::Out] double% lon2,
            [System::Runtime::InteropServices::Out] double% azi2,
            [System::Runtime::InteropServices::Out] double% m12);

        /**
         * See the documentation for GeodesicLine::Position.
         **********************************************************************/
        double Position(double s12,
            [System::Runtime::InteropServices::Out] double% lat2,
            [System::Runtime::InteropServices::Out] double% lon2,
            [System::Runtime::InteropServices::Out] double% azi2,
            [System::Runtime::InteropServices::Out] double% M12,
            [System::Runtime::InteropServices::Out] double% M21);

        /**
         * See the documentation for GeodesicLine::Position.
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
         *   be negative.
         * @param[out] lat2 latitude of point 2 (degrees).
         * @param[out] lon2 longitude of point 2 (degrees); requires that the
         *   GeodesicLine object was constructed with \e caps |=
         *   NETGeographicLib::Mask::LONGITUDE.
         * @param[out] azi2 (forward) azimuth at point 2 (degrees).
         * @param[out] s12 distance between point 1 and point 2 (meters); requires
         *   that the GeodesicLine object was constructed with \e caps |=
         *   NETGeographicLib::Mask::DISTANCE.
         * @param[out] m12 reduced length of geodesic (meters); requires that the
         *   GeodesicLine object was constructed with \e caps |=
         *   NETGeographicLib::Mask::REDUCEDLENGTH.
         * @param[out] M12 geodesic scale of point 2 relative to point 1
         *   (dimensionless); requires that the GeodesicLine object was constructed
         *   with \e caps |= NETGeographicLib::Mask::GEODESICSCALE.
         * @param[out] M21 geodesic scale of point 1 relative to point 2
         *   (dimensionless); requires that the GeodesicLine object was constructed
         *   with \e caps |= NETGeographicLib::Mask::GEODESICSCALE.
         * @param[out] S12 area under the geodesic (meters<sup>2</sup>); requires
         *   that the GeodesicLine object was constructed with \e caps |=
         *   NETGeographicLib::Mask::AREA.
         *
         * The values of \e lon2 and \e azi2 returned are in the range
         * [&minus;180&deg;, 180&deg;).
         *
         * Requesting a value which the GeodesicLine object is not capable of
         * computing is not an error; the corresponding argument will not be
         * altered.
         *
         * The following functions are overloaded versions of
         * GeodesicLine::ArcPosition which omit some of the output parameters.
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
         * See the documentation for GeodesicLine::ArcPosition.
         **********************************************************************/
        void ArcPosition(double a12,
            [System::Runtime::InteropServices::Out] double% lat2,
            [System::Runtime::InteropServices::Out] double% lon2);

        /**
         * See the documentation for GeodesicLine::ArcPosition.
         **********************************************************************/
        void ArcPosition(double a12,
            [System::Runtime::InteropServices::Out] double% lat2,
            [System::Runtime::InteropServices::Out] double% lon2,
            [System::Runtime::InteropServices::Out] double% azi2);

        /**
         * See the documentation for GeodesicLine::ArcPosition.
         **********************************************************************/
        void ArcPosition(double a12,
            [System::Runtime::InteropServices::Out] double% lat2,
            [System::Runtime::InteropServices::Out] double% lon2,
            [System::Runtime::InteropServices::Out] double% azi2,
            [System::Runtime::InteropServices::Out] double% s12);

        /**
         * See the documentation for GeodesicLine::ArcPosition.
         **********************************************************************/
        void ArcPosition(double a12,
            [System::Runtime::InteropServices::Out] double% lat2,
            [System::Runtime::InteropServices::Out] double% lon2,
            [System::Runtime::InteropServices::Out] double% azi2,
            [System::Runtime::InteropServices::Out] double% s12,
            [System::Runtime::InteropServices::Out] double% m12);

        /**
         * See the documentation for GeodesicLine::ArcPosition.
         **********************************************************************/
        void ArcPosition(double a12,
            [System::Runtime::InteropServices::Out] double% lat2,
            [System::Runtime::InteropServices::Out] double% lon2,
            [System::Runtime::InteropServices::Out] double% azi2,
            [System::Runtime::InteropServices::Out] double% s12,
            [System::Runtime::InteropServices::Out] double% M12,
            [System::Runtime::InteropServices::Out] double% M21);

        /**
         * See the documentation for GeodesicLine::ArcPosition.
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
         * The general position function.  GeodesicLine::Position and
         * GeodesicLine::ArcPosition are defined in terms of this function.
         *
         * @param[in] arcmode boolean flag determining the meaning of the second
         *   parameter; if arcmode is false, then the GeodesicLine object must have
         *   been constructed with \e caps |= GeodesicLine::DISTANCE_IN.
         * @param[in] s12_a12 if \e arcmode is false, this is the distance between
         *   point 1 and point 2 (meters); otherwise it is the arc length between
         *   point 1 and point 2 (degrees); it can be negative.
         * @param[in] outmask a bitor'ed combination of NETGeographicLib::Mask values
         *   specifying which of the following parameters should be set.
         * @param[out] lat2 latitude of point 2 (degrees).
         * @param[out] lon2 longitude of point 2 (degrees); requires that the
         *   GeodesicLine object was constructed with \e caps |=
         *   NETGeographicLib::Mask::LONGITUDE.
         * @param[out] azi2 (forward) azimuth at point 2 (degrees).
         * @param[out] s12 distance between point 1 and point 2 (meters); requires
         *   that the GeodesicLine object was constructed with \e caps |=
         *   NETGeographicLib::Mask::DISTANCE.
         * @param[out] m12 reduced length of geodesic (meters); requires that the
         *   GeodesicLine object was constructed with \e caps |=
         *   NETGeographicLib::Mask::REDUCEDLENGTH.
         * @param[out] M12 geodesic scale of point 2 relative to point 1
         *   (dimensionless); requires that the GeodesicLine object was constructed
         *   with \e caps |= NETGeographicLib::Mask::GEODESICSCALE.
         * @param[out] M21 geodesic scale of point 1 relative to point 2
         *   (dimensionless); requires that the GeodesicLine object was constructed
         *   with \e caps |= NETGeographicLib::Mask::GEODESICSCALE.
         * @param[out] S12 area under the geodesic (meters<sup>2</sup>); requires
         *   that the GeodesicLine object was constructed with \e caps |=
         *   NETGeographicLib::Mask::AREA.
         * @return \e a12 arc length of between point 1 and point 2 (degrees).
         *
         * The GeodesicLine::mask values possible for \e outmask are
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
         * Requesting a value which the GeodesicLine object is not capable of
         * computing is not an error; the corresponding argument will not be
         * altered.  Note, however, that the arc length is always computed and
         * returned as the function value.
         **********************************************************************/
        double GenPosition(bool arcmode, double s12_a12,
                    NETGeographicLib::Mask outmask,
                    [System::Runtime::InteropServices::Out] double% lat2,
                    [System::Runtime::InteropServices::Out] double% lon2,
                    [System::Runtime::InteropServices::Out] double% azi2,
                    [System::Runtime::InteropServices::Out] double% s12,
                    [System::Runtime::InteropServices::Out] double% m12,
                    [System::Runtime::InteropServices::Out] double% M12,
                    [System::Runtime::InteropServices::Out] double% M21,
                    [System::Runtime::InteropServices::Out] double% S12);

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
         *   the equator in a northward direction.
         **********************************************************************/
        property double EquatorialAzimuth { double get(); }

        /**
         * @return \e a1 the arc length (degrees) between the northward equatorial
         *   crossing and point 1.
         **********************************************************************/
        property double EquatorialArc { double get(); }

        /**
         * @return \e a the equatorial radius of the ellipsoid (meters).  This is
         *   the value inherited from the Geodesic object used in the constructor.
         **********************************************************************/
        property double MajorRadius { double get(); }

        /**
         * @return \e f the flattening of the ellipsoid.  This is the value
         *   inherited from the Geodesic object used in the constructor.
         **********************************************************************/
        property double Flattening { double get(); }

        /**
         * @return \e caps the computational capabilities that this object was
         *   constructed with.  LATITUDE and AZIMUTH are always included.
         **********************************************************************/
        NETGeographicLib::Mask Capabilities();

        /**
         * @param[in] testcaps a set of bitor'ed GeodesicLine::mask values.
         * @return true if the GeodesicLine object has all these capabilities.
         **********************************************************************/
        bool Capabilities(NETGeographicLib::Mask testcaps);
        ///@}
    };
} // namespace NETGeographicLib
