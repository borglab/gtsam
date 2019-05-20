#pragma once
/**
 * \file NETGeographicLib/TransverseMercatorExact.h
 * \brief Header for NETGeographicLib::TransverseMercatorExact class
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
   * \brief .NET wrapper for GeographicLib::TransverseMercatorExact.
   *
   * This class allows .NET applications to access GeographicLib::TransverseMercatorExact.
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
   * See GeographicLib::TransverseMercatorExact.cpp for more information on the
   * implementation.
   *
   * See \ref transversemercator for a discussion of this projection.
   *
   * C# Example:
   * \include example-TransverseMercatorExact.cs
   * Managed C++ Example:
   * \include example-TransverseMercatorExact.cpp
   * Visual Basic Example:
   * \include example-TransverseMercatorExact.vb
   *
   * <B>INTERFACE DIFFERENCES:</B><BR>
   * A default constructor is provided that assumes WGS84 parameters and
   * a UTM scale factor.
   *
   * The MajorRadius, Flattening, and CentralScale functions are
   * implemented as properties.
   **********************************************************************/
    public ref class TransverseMercatorExact
    {
        private:
        // a pointer to the unmanaged GeographicLib::TransverseMercatorExact.
        GeographicLib::TransverseMercatorExact* m_pTransverseMercatorExact;
        // the finalizer frees the unmanaged memory when the object is destroyed.
        !TransverseMercatorExact(void);
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
         * numeric_limits<double>::epsilon().  However, TransverseMercator treats the
         * sphere exactly.
         **********************************************************************/
        TransverseMercatorExact(double a, double f, double k0, bool extendp );

        /**
         * The default constructor assumes a WGS84 ellipsoid and a UTM scale
         * factor.
         **********************************************************************/
        TransverseMercatorExact();

        /**
         * The destructor calls the finalizer.
         **********************************************************************/
        ~TransverseMercatorExact()
        { this->!TransverseMercatorExact(); }

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
         * TransverseMercatorExact::Forward without returning the convergence and
         * scale.
         **********************************************************************/
        void Forward(double lon0, double lat, double lon,
                     [System::Runtime::InteropServices::Out] double% x,
                     [System::Runtime::InteropServices::Out] double% y);

        /**
         * TransverseMercatorExact::Reverse without returning the convergence and
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
         * @return \e k0 central scale for the projection.  This is the value of \e
         *   k0 used in the constructor and is the scale on the central meridian.
         **********************************************************************/
        property double CentralScale { double get(); }
        ///@}
    };
} // namespace NETGeographicLib
