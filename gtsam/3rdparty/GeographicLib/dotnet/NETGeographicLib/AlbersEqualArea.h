#pragma once
/**
 * \file NETGeographicLib/AlbersEqualArea.h
 * \brief Header for NETGeographicLib::AlbersEqualArea class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

namespace NETGeographicLib
{
  /**
   * \brief .NET Wrapper for GeographicLib::AlbersEqualArea.
   *
   * This class allows .NET applications to access
   * GeographicLib::AlbersEqualArea
   *
   * Implementation taken from the report,
   * - J. P. Snyder,
   *   <a href="http://pubs.er.usgs.gov/usgspubs/pp/pp1395"> Map Projections: A
   *   Working Manual</a>, USGS Professional Paper 1395 (1987),
   *   pp. 101--102.
   *
   * This is a implementation of the equations in Snyder except that divided
   * differences will be [have been] used to transform the expressions into
   * ones which may be evaluated accurately.  [In this implementation, the
   * projection correctly becomes the cylindrical equal area or the azimuthal
   * equal area projection when the standard latitude is the equator or a
   * pole.]
   *
   * The ellipsoid parameters, the standard parallels, and the scale on the
   * standard parallels are set in the constructor.  Internally, the case with
   * two standard parallels is converted into a single standard parallel, the
   * latitude of minimum azimuthal scale, with an azimuthal scale specified on
   * this parallel.  This latitude is also used as the latitude of origin which
   * is returned by AlbersEqualArea::OriginLatitude.  The azimuthal scale on
   * the latitude of origin is given by AlbersEqualArea::CentralScale.  The
   * case with two standard parallels at opposite poles is singular and is
   * disallowed.  The central meridian (which is a trivial shift of the
   * longitude) is specified as the \e lon0 argument of the
   * AlbersEqualArea::Forward and AlbersEqualArea::Reverse functions.
   * AlbersEqualArea::Forward and AlbersEqualArea::Reverse also return the
   * meridian convergence, &gamma;, and azimuthal scale, \e k.  A small square
   * aligned with the cardinal directions is projected to a rectangle with
   * dimensions \e k (in the E-W direction) and 1/\e k (in the N-S direction).
   * The E-W sides of the rectangle are oriented &gamma; degrees
   * counter-clockwise from the \e x axis.  There is no provision in this class
   * for specifying a false easting or false northing or a different latitude
   * of origin.
   *
   * C# Example:
   * \include example-AlbersEqualArea.cs
   * Managed C++ Example:
   * \include example-AlbersEqualArea.cpp
   * Visual Basic Example:
   * \include example-AlbersEqualArea.vb
   *
   * <B>INTERFACE DIFFERENCES:</B><BR>
   * A constructor has been provided that creates the standard projections.
   *
   * The MajorRadius, Flattening, OriginLatitude, and CentralScale functions
   * are implemented as properties.
   **********************************************************************/
    public ref class AlbersEqualArea
    {
    private:
        // pointer to the unmanaged GeographicLib::AlbersEqualArea
        GeographicLib::AlbersEqualArea* m_pAlbersEqualArea;

        // Frees the unmanaged m_pAlbersEqualArea when object is destroyed.
        !AlbersEqualArea();
    public:
        /**
         Standard AlbersEqualAreaProjections that assume the WGS84 ellipsoid.
         *********************************************************************/
        enum class StandardTypes
        {
            CylindricalEqualArea,       //!< cylindrical equal area projection (stdlat = 0, and \e k0 = 1)
            AzimuthalEqualAreaNorth,    //!< Lambert azimuthal equal area projection (stdlat = 90&deg;, and \e k0 = 1)
            AzimuthalEqualAreaSouth     //!< Lambert azimuthal equal area projection (stdlat = &minus;90&deg;, and \e k0 = 1)
        };

        //! \brief Destructor
        ~AlbersEqualArea() { this->!AlbersEqualArea(); }

        /**!
         * Constructor for one of the standard types.
         * @param[in] type The desired standard type.
         **********************************************************************/
        AlbersEqualArea( StandardTypes type );

        /**
         * Constructor with a single standard parallel.
         *
         * @param[in] a equatorial radius of ellipsoid (meters).
         * @param[in] f flattening of ellipsoid.  Setting \e f = 0 gives a sphere.
         *   Negative \e f gives a prolate ellipsoid.
         * @param[in] stdlat standard parallel (degrees), the circle of tangency.
         * @param[in] k0 azimuthal scale on the standard parallel.
         * @exception GeographicErr if \e a, (1 &minus; \e f ) \e a, or \e k0 is
         *   not positive.
         * @exception GeographicErr if \e stdlat is not in [&minus;90&deg;,
         *   90&deg;].
         **********************************************************************/
        AlbersEqualArea(double a, double f, double stdlat, double k0);

        /**
         * Constructor with two standard parallels.
         *
         * @param[in] a equatorial radius of ellipsoid (meters).
         * @param[in] f flattening of ellipsoid.  Setting \e f = 0 gives a sphere.
         *   Negative \e f gives a prolate ellipsoid.
         * @param[in] stdlat1 first standard parallel (degrees).
         * @param[in] stdlat2 second standard parallel (degrees).
         * @param[in] k1 azimuthal scale on the standard parallels.
         * @exception GeographicErr if \e a, (1 &minus; \e f ) \e a, or \e k1 is
         *   not positive.
         * @exception GeographicErr if \e stdlat1 or \e stdlat2 is not in
         *   [&minus;90&deg;, 90&deg;], or if \e stdlat1 and \e stdlat2 are
         *   opposite poles.
         **********************************************************************/
        AlbersEqualArea(double a, double f, double stdlat1, double stdlat2, double k1);

        /**
         * Constructor with two standard parallels specified by sines and cosines.
         *
         * @param[in] a equatorial radius of ellipsoid (meters).
         * @param[in] f flattening of ellipsoid.  Setting \e f = 0 gives a sphere.
         *   Negative \e f gives a prolate ellipsoid.
         * @param[in] sinlat1 sine of first standard parallel.
         * @param[in] coslat1 cosine of first standard parallel.
         * @param[in] sinlat2 sine of second standard parallel.
         * @param[in] coslat2 cosine of second standard parallel.
         * @param[in] k1 azimuthal scale on the standard parallels.
         * @exception GeographicErr if \e a, (1 &minus; \e f ) \e a, or \e k1 is
         *   not positive.
         * @exception GeographicErr if \e stdlat1 or \e stdlat2 is not in
         *   [&minus;90&deg;, 90&deg;], or if \e stdlat1 and \e stdlat2 are
         *   opposite poles.
         *
         * This allows parallels close to the poles to be specified accurately.
         * This routine computes the latitude of origin and the azimuthal scale at
         * this latitude.  If \e dlat = abs(\e lat2 &minus; \e lat1) &le; 160&deg;,
         * then the error in the latitude of origin is less than 4.5 &times;
         * 10<sup>&minus;14</sup>d;.
         **********************************************************************/
        AlbersEqualArea(double a, double f,
                        double sinlat1, double coslat1,
                        double sinlat2, double coslat2,
                        double k1);

        /**
         * Set the azimuthal scale for the projection.
         *
         * @param[in] lat (degrees).
         * @param[in] k azimuthal scale at latitude \e lat (default 1).
         * @exception GeographicErr \e k is not positive.
         * @exception GeographicErr if \e lat is not in (&minus;90&deg;,
         *   90&deg;).
         *
         * This allows a "latitude of conformality" to be specified.
         **********************************************************************/
        void SetScale(double lat, double k);

        /**
         * Forward projection, from geographic to Lambert conformal conic.
         *
         * @param[in] lon0 central meridian longitude (degrees).
         * @param[in] lat latitude of point (degrees).
         * @param[in] lon longitude of point (degrees).
         * @param[out] x easting of point (meters).
         * @param[out] y northing of point (meters).
         * @param[out] gamma meridian convergence at point (degrees).
         * @param[out] k azimuthal scale of projection at point; the radial
         *   scale is the 1/\e k.
         *
         * The latitude origin is given by AlbersEqualArea::LatitudeOrigin().
         * No false easting or northing is added and \e lat should be in the
         * range [&minus;90&deg;, 90&deg;].  The values of \e x and \e y
         * returned for points which project to infinity (i.e., one or both of
         * the poles) will be large but finite.
         **********************************************************************/
        void Forward(double lon0, double lat, double lon,
                     [System::Runtime::InteropServices::Out] double% x,
                     [System::Runtime::InteropServices::Out] double% y,
                     [System::Runtime::InteropServices::Out] double% gamma,
                     [System::Runtime::InteropServices::Out] double% k);

        /**
         * Reverse projection, from Lambert conformal conic to geographic.
         *
         * @param[in] lon0 central meridian longitude (degrees).
         * @param[in] x easting of point (meters).
         * @param[in] y northing of point (meters).
         * @param[out] lat latitude of point (degrees).
         * @param[out] lon longitude of point (degrees).
         * @param[out] gamma meridian convergence at point (degrees).
         * @param[out] k azimuthal scale of projection at point; the radial
         *   scale is the 1/\e k.
         *
         * The latitude origin is given by AlbersEqualArea::LatitudeOrigin().
         * No false easting or northing is added.  The value of \e lon returned
         * is in the range [&minus;180&deg;, 180&deg;).  The value of \e lat
         * returned is in the range [&minus;90&deg;, 90&deg;].  If the input
         * point is outside the legal projected space the nearest pole is
         * returned.
         **********************************************************************/
        void Reverse(double lon0, double x, double y,
                     [System::Runtime::InteropServices::Out] double% lat,
                     [System::Runtime::InteropServices::Out] double% lon,
                     [System::Runtime::InteropServices::Out] double% gamma,
                     [System::Runtime::InteropServices::Out] double% k);

        /**
         * AlbersEqualArea::Forward without returning the convergence and
         * scale.
         **********************************************************************/
        void Forward(double lon0, double lat, double lon,
                     [System::Runtime::InteropServices::Out] double% x,
                     [System::Runtime::InteropServices::Out] double% y);

        /**
         * AlbersEqualArea::Reverse without returning the convergence and
         * scale.
         **********************************************************************/
        void Reverse(double lon0, double x, double y,
                     [System::Runtime::InteropServices::Out] double% lat,
                     [System::Runtime::InteropServices::Out] double% lon);

        /** \name Inspector functions
         **********************************************************************/
        ///@{
        /**
         * @return \e a the equatorial radius of the ellipsoid (meters).  This is
         *   the value used in the constructor.
         **********************************************************************/
        property double MajorRadius { double get(); }

        /**
         * @return \e f the flattening of the ellipsoid.  This is the value used in
         *   the constructor.
         **********************************************************************/
        property double Flattening { double get(); }

        /**
         * @return latitude of the origin for the projection (degrees).
         *
         * This is the latitude of minimum azimuthal scale and equals the \e stdlat
         * in the 1-parallel constructor and lies between \e stdlat1 and \e stdlat2
         * in the 2-parallel constructors.
         **********************************************************************/
        property double OriginLatitude { double get(); }

        /**
         * @return central scale for the projection.  This is the azimuthal scale
         *   on the latitude of origin.
         **********************************************************************/
        property double CentralScale { double get(); }
        ///@}
    };
} // namespace NETGeographic
