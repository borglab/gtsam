#pragma once
/**
 * \file NETGeographicLib/TransverseMercator.h
 * \brief Header for NETGeographicLib::TransverseMercator class
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
   * \brief .NET wrapper for GeographicLib::TransverseMercator.
   *
   * This class allows .NET applications to access GeographicLib::TransverseMercator.
   *
   * This uses Kr&uuml;ger's method which evaluates the projection and its
   * inverse in terms of a series.  See
   *  - L. Kr&uuml;ger,
   *    <a href="https://doi.org/10.2312/GFZ.b103-krueger28"> Konforme
   *    Abbildung des Erdellipsoids in der Ebene</a> (Conformal mapping of the
   *    ellipsoidal earth to the plane), Royal Prussian Geodetic Institute, New
   *    Series 52, 172 pp. (1912).
   *  - C. F. F. Karney,
   *    <a href="https://doi.org/10.1007/s00190-011-0445-3">
   *    Transverse Mercator with an accuracy of a few nanometers,</a>
   *    J. Geodesy 85(8), 475--485 (Aug. 2011);
   *    preprint
   *    <a href="https://arxiv.org/abs/1002.1417">arXiv:1002.1417</a>.
   *
   * Kr&uuml;ger's method has been extended from 4th to 6th order.  The maximum
   * error is 5 nm (5 nanometers), ground distance, for all positions within 35
   * degrees of the central meridian.  The error in the convergence is 2
   * &times; 10<sup>&minus;15</sup>&quot; and the relative error in the scale
   * is 6 &minus; 10<sup>&minus;12</sup>%%.  See Sec. 4 of
   * <a href="https://arxiv.org/abs/1002.1417">arXiv:1002.1417</a> for details.
   * The speed penalty in going to 6th order is only about 1%.
   * TransverseMercatorExact is an alternative implementation of the projection
   * using exact formulas which yield accurate (to 8 nm) results over the
   * entire ellipsoid.
   *
   * The ellipsoid parameters and the central scale are set in the constructor.
   * The central meridian (which is a trivial shift of the longitude) is
   * specified as the \e lon0 argument of the TransverseMercator::Forward and
   * TransverseMercator::Reverse functions.  The latitude of origin is taken to
   * be the equator.  There is no provision in this class for specifying a
   * false easting or false northing or a different latitude of origin.
   * However these are can be simply included by the calling function.  For
   * example, the UTMUPS class applies the false easting and false northing for
   * the UTM projections.  A more complicated example is the British National
   * Grid (<a href="http://www.spatialreference.org/ref/epsg/7405/">
   * EPSG:7405</a>) which requires the use of a latitude of origin.  This is
   * implemented by the GeographicLib::OSGB class.
   *
   * See GeographicLib::TransverseMercator.cpp for more information on the
   * implementation.
   *
   * See \ref transversemercator for a discussion of this projection.
   *
   * C# Example:
   * \include example-TransverseMercator.cs
   * Managed C++ Example:
   * \include example-TransverseMercator.cpp
   * Visual Basic Example:
   * \include example-TransverseMercator.vb
   *
   * <B>INTERFACE DIFFERENCES:</B><BR>
   * A default constructor is provided that assumes WGS84 parameters and
   * a UTM scale factor.
   *
   * The MajorRadius, Flattening, and CentralScale functions are
   * implemented as properties.
   **********************************************************************/
    public ref class TransverseMercator
    {
        private:
        // pointer to the unmanaged GeographicLib::TransverseMercator.
        const GeographicLib::TransverseMercator* m_pTransverseMercator;
        // the finalizer frees the unmanaged memory when the object is destroyed.
        !TransverseMercator(void);
    public:
        /**
         * Constructor for a ellipsoid with
         *
         * @param[in] a equatorial radius (meters).
         * @param[in] f flattening of ellipsoid.  Setting \e f = 0 gives a sphere.
         *   Negative \e f gives a prolate ellipsoid.
         * @param[in] k0 central scale factor.
         * @exception GeographicErr if \e a, (1 &minus; \e f ) \e a, or \e k0 is
         *   not positive.
         **********************************************************************/
        TransverseMercator(double a, double f, double k0);

        /**
         * The default constructor assumes a WGS84 ellipsoid and a UTM scale
         * factor.
         **********************************************************************/
        TransverseMercator();

        /**
         * The destructor calls the finalizer.
         **********************************************************************/
        ~TransverseMercator()
        { this->!TransverseMercator(); }

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
        void Forward(double lon0, double lat, double lon,
                     [System::Runtime::InteropServices::Out] double% x,
                     [System::Runtime::InteropServices::Out] double% y,
                     [System::Runtime::InteropServices::Out] double% gamma,
                     [System::Runtime::InteropServices::Out] double% k);

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
         * No false easting or northing is added.  The value of \e lon returned
         * is in the range [&minus;180&deg;, 180&deg;).
         **********************************************************************/
        void Reverse(double lon0, double x, double y,
                     [System::Runtime::InteropServices::Out] double% lat,
                     [System::Runtime::InteropServices::Out] double% lon,
                     [System::Runtime::InteropServices::Out] double% gamma,
                     [System::Runtime::InteropServices::Out] double% k);

        /**
         * TransverseMercator::Forward without returning the convergence and scale.
         **********************************************************************/
        void Forward(double lon0, double lat, double lon,
                     [System::Runtime::InteropServices::Out] double% x,
                     [System::Runtime::InteropServices::Out] double% y);

        /**
         * TransverseMercator::Reverse without returning the convergence and scale.
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
         * @return \e k0 central scale for the projection.  This is the value of \e
         *   k0 used in the constructor and is the scale on the central meridian.
         **********************************************************************/
        property double CentralScale { double get(); }
        ///@}
    };
} // namespace NETGeographicLib
