#pragma once
/**
 * \file NETGeographicLib/LambertConformalConic.h
 * \brief Header for NETGeographicLib::LambertConformalConic class
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
   * \brief .NET wrapper for GeographicLib::LambertConformalConic.
   *
   * This class allows .NET applications to access GeographicLib::LambertConformalConic.
   *
   * Implementation taken from the report,
   * - J. P. Snyder,
   *   <a href="http://pubs.er.usgs.gov/usgspubs/pp/pp1395"> Map Projections: A
   *   Working Manual</a>, USGS Professional Paper 1395 (1987),
   *   pp. 107--109.
   *
   * This is a implementation of the equations in Snyder except that divided
   * differences have been used to transform the expressions into ones which
   * may be evaluated accurately and that Newton's method is used to invert the
   * projection.  In this implementation, the projection correctly becomes the
   * Mercator projection or the polar stereographic projection when the
   * standard latitude is the equator or a pole.  The accuracy of the
   * projections is about 10 nm (10 nanometers).
   *
   * The ellipsoid parameters, the standard parallels, and the scale on the
   * standard parallels are set in the constructor.  Internally, the case with
   * two standard parallels is converted into a single standard parallel, the
   * latitude of tangency (also the latitude of minimum scale), with a scale
   * specified on this parallel.  This latitude is also used as the latitude of
   * origin which is returned by LambertConformalConic::OriginLatitude.  The
   * scale on the latitude of origin is given by
   * LambertConformalConic::CentralScale.  The case with two distinct standard
   * parallels where one is a pole is singular and is disallowed.  The central
   * meridian (which is a trivial shift of the longitude) is specified as the
   * \e lon0 argument of the LambertConformalConic::Forward and
   * LambertConformalConic::Reverse functions.  There is no provision in this
   * class for specifying a false easting or false northing or a different
   * latitude of origin.  However these are can be simply included by the
   * calling function.  For example the Pennsylvania South state coordinate
   * system (<a href="http://www.spatialreference.org/ref/epsg/3364/">
   * EPSG:3364</a>) is obtained by:
   * C# Example:
   * \include example-LambertConformalConic.cs
   * Managed C++ Example:
   * \include example-LambertConformalConic.cpp
   * Visual Basic Example:
   * \include example-LambertConformalConic.vb
   *
   * <B>INTERFACE DIFFERENCES:</B><BR>
   * A default constructor has been provided that assumes a Mercator
   * projection.
   *
   * The MajorRadius, Flattening, OriginLatitude, and CentralScale
   * functions are implemented as properties.
   **********************************************************************/
    public ref class LambertConformalConic
    {
        private:
        // Pointer to the unmanaged GeographicLib::LambertConformalConic.
        GeographicLib::LambertConformalConic* m_pLambertConformalConic;

        // the finalizer frres the unmanaged memory when the object is destroyed.
        !LambertConformalConic(void);
    public:

        /**
         * Constructor with a single standard parallel.
         *
         * @param[in] a equatorial radius of ellipsoid (meters).
         * @param[in] f flattening of ellipsoid.  Setting \e f = 0 gives a sphere.
         *   Negative \e f gives a prolate ellipsoid.
         * @param[in] stdlat standard parallel (degrees), the circle of tangency.
         * @param[in] k0 scale on the standard parallel.
         * @exception GeographicErr if \e a, (1 &minus; \e f ) \e a, or \e k0 is
         *   not positive.
         * @exception GeographicErr if \e stdlat is not in [&minus;90&deg;,
         *   90&deg;].
         **********************************************************************/
        LambertConformalConic(double a, double f, double stdlat, double k0);

        /**
         * Constructor with two standard parallels.
         *
         * @param[in] a equatorial radius of ellipsoid (meters).
         * @param[in] f flattening of ellipsoid.  Setting \e f = 0 gives a sphere.
         *   Negative \e f gives a prolate ellipsoid.
         * @param[in] stdlat1 first standard parallel (degrees).
         * @param[in] stdlat2 second standard parallel (degrees).
         * @param[in] k1 scale on the standard parallels.
         * @exception GeographicErr if \e a, (1 &minus; \e f ) \e a, or \e k1 is
         *   not positive.
         * @exception GeographicErr if \e stdlat1 or \e stdlat2 is not in
         *   [&minus;90&deg;, 90&deg;], or if either \e stdlat1 or \e
         *   stdlat2 is a pole and \e stdlat1 is not equal \e stdlat2.
         **********************************************************************/
        LambertConformalConic(double a, double f, double stdlat1, double stdlat2, double k1);

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
         * @param[in] k1 scale on the standard parallels.
         * @exception GeographicErr if \e a, (1 &minus; \e f ) \e a, or \e k1 is
         *   not positive.
         * @exception GeographicErr if \e stdlat1 or \e stdlat2 is not in
         *   [&minus;90&deg;, 90&deg;], or if either \e stdlat1 or \e
         *   stdlat2 is a pole and \e stdlat1 is not equal \e stdlat2.
         *
         * This allows parallels close to the poles to be specified accurately.
         * This routine computes the latitude of origin and the scale at this
         * latitude.  In the case where \e lat1 and \e lat2 are different, the
         * errors in this routines are as follows: if \e dlat = abs(\e lat2 &minus;
         * \e lat1) &le; 160&deg; and max(abs(\e lat1), abs(\e lat2)) &le; 90
         * &minus; min(0.0002, 2.2 &times; 10<sup>&minus;6</sup>(180 &minus; \e
         * dlat), 6 &times 10<sup>&minus;8</sup> <i>dlat</i><sup>2</sup>) (in
         * degrees), then the error in the latitude of origin is less than 4.5
         * &times; 10<sup>&minus;14</sup>d and the relative error in the scale is
         * less than 7 &times; 10<sup>&minus;15</sup>.
         **********************************************************************/
        LambertConformalConic(double a, double f,
                              double sinlat1, double coslat1,
                              double sinlat2, double coslat2,
                              double k1);

        /**
         * The default constructor assumes a Mercator projection.
         **********************************************************************/
        LambertConformalConic();

        /**
         * The destructor calls the finalizer.
         **********************************************************************/
        ~LambertConformalConic()
        { this->!LambertConformalConic(); }

        /**
         * Set the scale for the projection.
         *
         * @param[in] lat (degrees).
         * @param[in] k scale at latitude \e lat (default 1).
         * @exception GeographicErr \e k is not positive.
         * @exception GeographicErr if \e lat is not in [&minus;90&deg;,
         *   90&deg;].
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
         * @param[out] k scale of projection at point.
         *
         * The latitude origin is given by
         * LambertConformalConic::LatitudeOrigin().  No false easting or
         * northing is added and \e lat should be in the range [&minus;90&deg;,
         * 90&deg;].  The error in the projection is less than about 10 nm (10
         * nanometers), true distance, and the errors in the meridian
         * convergence and scale are consistent with this.  The values of \e x
         * and \e y returned for points which project to infinity (i.e., one or
         * both of the poles) will be large but finite.
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
         * @param[out] k scale of projection at point.
         *
         * The latitude origin is given by
         * LambertConformalConic::LatitudeOrigin().  No false easting or
         * northing is added.  The value of \e lon returned is in the range
         * [&minus;180&deg;, 180&deg;).  The error in the projection is less
         * than about 10 nm (10 nanometers), true distance, and the errors in
         * the meridian convergence and scale are consistent with this.
         **********************************************************************/
        void Reverse(double lon0, double x, double y,
                     [System::Runtime::InteropServices::Out] double% lat,
                     [System::Runtime::InteropServices::Out] double% lon,
                     [System::Runtime::InteropServices::Out] double% gamma,
                     [System::Runtime::InteropServices::Out] double% k);

        /**
         * LambertConformalConic::Forward without returning the convergence and
         * scale.
         **********************************************************************/
        void Forward(double lon0, double lat, double lon,
                     [System::Runtime::InteropServices::Out] double% x,
                     [System::Runtime::InteropServices::Out] double% y);

        /**
         * LambertConformalConic::Reverse without returning the convergence and
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
         * @return \e f the flattening of the ellipsoid.  This is the
         *   value used in the constructor.
         **********************************************************************/
        property double Flattening { double get(); }

        /**
         * @return latitude of the origin for the projection (degrees).
         *
         * This is the latitude of minimum scale and equals the \e stdlat in the
         * 1-parallel constructor and lies between \e stdlat1 and \e stdlat2 in the
         * 2-parallel constructors.
         **********************************************************************/
        property double OriginLatitude { double get(); }

        /**
         * @return central scale for the projection.  This is the scale on the
         *   latitude of origin.
         **********************************************************************/
        property double CentralScale { double get(); }
        ///@}
    };
} //namespace NETGeographicLib
