/**
 * \file NETGeographicLib/Geocentric.h
 * \brief Header for NETGeographicLib::Geocentric class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#pragma once

namespace NETGeographicLib
{
  /**
   * \brief .NET wrapper for GeographicLib::Geocentric.
   *
   * This class allows .NET applications to access GeographicLib::Geocentric.
   *
   * Convert between geodetic coordinates latitude = \e lat, longitude = \e
   * lon, height = \e h (measured vertically from the surface of the ellipsoid)
   * to geocentric coordinates (\e X, \e Y, \e Z).  The origin of geocentric
   * coordinates is at the center of the earth.  The \e Z axis goes thru the
   * north pole, \e lat = 90&deg;.  The \e X axis goes thru \e lat = 0,
   * \e lon = 0.  %Geocentric coordinates are also known as earth centered,
   * earth fixed (ECEF) coordinates.
   *
   * The conversion from geographic to geocentric coordinates is
   * straightforward.  For the reverse transformation we use
   * - H. Vermeille,
   *   <a href="https://doi.org/10.1007/s00190-002-0273-6"> Direct
   *   transformation from geocentric coordinates to geodetic coordinates</a>,
   *   J. Geodesy 76, 451--454 (2002).
   * .
   * Several changes have been made to ensure that the method returns accurate
   * results for all finite inputs (even if \e h is infinite).  The changes are
   * described in Appendix B of
   * - C. F. F. Karney,
   *   <a href="https://arxiv.org/abs/1102.1215v1">Geodesics
   *   on an ellipsoid of revolution</a>,
   *   Feb. 2011;
   *   preprint
   *   <a href="https://arxiv.org/abs/1102.1215v1">arxiv:1102.1215v1</a>.
   * .
   * See \ref geocentric for more information.
   *
   * The errors in these routines are close to round-off.  Specifically, for
   * points within 5000 km of the surface of the ellipsoid (either inside or
   * outside the ellipsoid), the error is bounded by 7 nm (7 nanometers) for
   * the WGS84 ellipsoid.  See \ref geocentric for further information on the
   * errors.
   *
   * C# Example:
   * \include example-Geocentric.cs
   * Managed C++ Example:
   * \include example-Geocentric.cpp
   * Visual Basic Example:
   * \include example-Geocentric.vb
   *
   * <B>INTERFACE DIFFERENCES:</B><BR>
   * A default constructor is provided that assumes WGS84 parameters.
   *
   * The MajorRadius and Flattening functions are implemented as properties.
   *
   * The Forward and Reverse functions return rotation matrices as 2D,
   * 3 &times; 3 arrays rather than vectors.
   **********************************************************************/
    public ref class Geocentric
    {
    private:
        // pointer to the unmanaged GeographicLib::Geocentric
        const GeographicLib::Geocentric* m_pGeocentric;

        // The finalizer frees unmanaged memory when the object is destroyed.
        !Geocentric();
    public:
        /**
         * Constructor for a ellipsoid with
         *
         * @param[in] a equatorial radius (meters).
         * @param[in] f flattening of ellipsoid.  Setting \e f = 0 gives a sphere.
         *   Negative \e f gives a prolate ellipsoid.
         * @exception GeographicErr if \e a or (1 &minus; \e f ) \e a is not
         *   positive.
         **********************************************************************/
        Geocentric(double a, double f);

        /**
         * A default constructor which assumes WGS84.
         **********************************************************************/
        Geocentric();

        /**
         * A constructor that is initialized from an unmanaged
         * GeographicLib::Geocentric.  For internal use only.
         * @param[in] g An existing GeographicLib::Geocentric.
         **********************************************************************/
        Geocentric( const GeographicLib::Geocentric& g );

        /**
         * The destructor calls the finalizer.
         **********************************************************************/
        ~Geocentric()
        { this->!Geocentric(); }

        /**
         * Convert from geodetic to geocentric coordinates.
         *
         * @param[in] lat latitude of point (degrees).
         * @param[in] lon longitude of point (degrees).
         * @param[in] h height of point above the ellipsoid (meters).
         * @param[out] X geocentric coordinate (meters).
         * @param[out] Y geocentric coordinate (meters).
         * @param[out] Z geocentric coordinate (meters).
         *
         * \e lat should be in the range [&minus;90&deg;, 90&deg;].
         **********************************************************************/
        void Forward(double lat, double lon, double h,
            [System::Runtime::InteropServices::Out] double% X,
            [System::Runtime::InteropServices::Out] double% Y,
            [System::Runtime::InteropServices::Out] double% Z);

        /**
         * Convert from geodetic to geocentric coordinates and return rotation
         * matrix.
         *
         * @param[in] lat latitude of point (degrees).
         * @param[in] lon longitude of point (degrees).
         * @param[in] h height of point above the ellipsoid (meters).
         * @param[out] X geocentric coordinate (meters).
         * @param[out] Y geocentric coordinate (meters).
         * @param[out] Z geocentric coordinate (meters).
         * @param[out] M a 3 &times; 3 rotation matrix.
         *
         * Let \e v be a unit vector located at (\e lat, \e lon, \e h).  We can
         * express \e v as \e column vectors in one of two ways
         * - in east, north, up coordinates (where the components are relative to a
         *   local coordinate system at (\e lat, \e lon, \e h)); call this
         *   representation \e v1.
         * - in geocentric \e X, \e Y, \e Z coordinates; call this representation
         *   \e v0.
         * .
         * Then we have \e v0 = \e M &sdot; \e v1.
         **********************************************************************/
        void Forward(double lat, double lon, double h,
            [System::Runtime::InteropServices::Out] double% X,
            [System::Runtime::InteropServices::Out] double% Y,
            [System::Runtime::InteropServices::Out] double% Z,
            [System::Runtime::InteropServices::Out] array<double,2>^% M);

        /**
         * Convert from geocentric to geodetic to coordinates.
         *
         * @param[in] X geocentric coordinate (meters).
         * @param[in] Y geocentric coordinate (meters).
         * @param[in] Z geocentric coordinate (meters).
         * @param[out] lat latitude of point (degrees).
         * @param[out] lon longitude of point (degrees).
         * @param[out] h height of point above the ellipsoid (meters).
         *
         * In general there are multiple solutions and the result which maximizes
         * \e h is returned.  If there are still multiple solutions with different
         * latitudes (applies only if \e Z = 0), then the solution with \e lat > 0
         * is returned.  If there are still multiple solutions with different
         * longitudes (applies only if \e X = \e Y = 0) then \e lon = 0 is
         * returned.  The value of \e h returned satisfies \e h &ge; &minus; \e a
         * (1 &minus; <i>e</i><sup>2</sup>) / sqrt(1 &minus; <i>e</i><sup>2</sup>
         * sin<sup>2</sup>\e lat).  The value of \e lon returned is in the range
         * [&minus;180&deg;, 180&deg;).
         **********************************************************************/
        void Reverse(double X, double Y, double Z,
            [System::Runtime::InteropServices::Out] double% lat,
            [System::Runtime::InteropServices::Out] double% lon,
            [System::Runtime::InteropServices::Out] double% h);

        /**
         * Convert from geocentric to geodetic to coordinates.
         *
         * @param[in] X geocentric coordinate (meters).
         * @param[in] Y geocentric coordinate (meters).
         * @param[in] Z geocentric coordinate (meters).
         * @param[out] lat latitude of point (degrees).
         * @param[out] lon longitude of point (degrees).
         * @param[out] h height of point above the ellipsoid (meters).
         * @param[out] M a 3 &times; 3 rotation matrix.
         *
         * Let \e v be a unit vector located at (\e lat, \e lon, \e h).  We can
         * express \e v as \e column vectors in one of two ways
         * - in east, north, up coordinates (where the components are relative to a
         *   local coordinate system at (\e lat, \e lon, \e h)); call this
         *   representation \e v1.
         * - in geocentric \e X, \e Y, \e Z coordinates; call this representation
         *   \e v0.
         * .
         * Then we have \e v1 = \e M<sup>T</sup> &sdot; \e v0, where \e
         * M<sup>T</sup> is the transpose of \e M.
         **********************************************************************/
        void Reverse(double X, double Y, double Z,
            [System::Runtime::InteropServices::Out] double% lat,
            [System::Runtime::InteropServices::Out] double% lon,
            [System::Runtime::InteropServices::Out] double% h,
            [System::Runtime::InteropServices::Out] array<double,2>^% M);

        /** \name Inspector functions
         **********************************************************************/
        ///@{
        /**
         * @return a pointer to the unmanaged GeographicLib::Geocentric.
         **********************************************************************/
        System::IntPtr^ GetUnmanaged();

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
        ///@}
    };
} // namespace NETGeographicLib
