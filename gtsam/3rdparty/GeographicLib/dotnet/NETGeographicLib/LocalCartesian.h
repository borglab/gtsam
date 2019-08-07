#pragma once
/**
 * \file NETGeographicLib/LocalCartesian.h
 * \brief Header for NETGeographicLib::LocalCartesian class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

namespace NETGeographicLib
{
    ref class Geocentric;
  /**
   * \brief .NET wrapper for GeographicLib::LocalCartesian.
   *
   * This class allows .NET applications to access GeographicLib::LocalCartesian.
   *
   * Convert between geodetic coordinates latitude = \e lat, longitude = \e
   * lon, height = \e h (measured vertically from the surface of the ellipsoid)
   * to local cartesian coordinates (\e x, \e y, \e z).  The origin of local
   * cartesian coordinate system is at \e lat = \e lat0, \e lon = \e lon0, \e h
   * = \e h0. The \e z axis is normal to the ellipsoid; the \e y axis points
   * due north.  The plane \e z = - \e h0 is tangent to the ellipsoid.
   *
   * The conversions all take place via geocentric coordinates using a
   * Geocentric object.
   *
   * C# Example:
   * \include example-LocalCartesian.cs
   * Managed C++ Example:
   * \include example-LocalCartesian.cpp
   * Visual Basic Example:
   * \include example-LocalCartesian.vb
   *
   * <B>INTERFACE DIFFERENCES:</B><BR>
   * Constructors have been provided that assume WGS84 parameters.
   *
   * The following functions are implemented as properties:
   * LatitudeOrigin, LongitudeOrigin, HeightOrigin, MajorRadius,
   * and Flattening.
   *
   * The rotation matrices returned by the Forward and Reverse functions
   * are 2D, 3 &times; 3 arrays rather than vectors.
   **********************************************************************/
    public ref class LocalCartesian
    {
        private:
        // the pointer to the GeographicLib::LocalCartesian.
        GeographicLib::LocalCartesian* m_pLocalCartesian;

        // the finalizer frees the unmanaged memory when the object is destroyed.
        !LocalCartesian(void);
    public:

        /**
         * Constructor setting the origin.
         *
         * @param[in] lat0 latitude at origin (degrees).
         * @param[in] lon0 longitude at origin (degrees).
         * @param[in] h0 height above ellipsoid at origin (meters); default 0.
         * @param[in] earth Geocentric object for the transformation; default
         *   Geocentric::WGS84.
         *
         * \e lat0 should be in the range [&minus;90&deg;, 90&deg;].
         **********************************************************************/
        LocalCartesian(double lat0, double lon0, double h0,
                       Geocentric^ earth );

        /**
         * Constructor setting the origin and assuming a WGS84 ellipsoid.
         *
         * @param[in] lat0 latitude at origin (degrees).
         * @param[in] lon0 longitude at origin (degrees).
         * @param[in] h0 height above ellipsoid at origin (meters); default 0.
         *
         * \e lat0 should be in the range [&minus;90&deg;, 90&deg;].
         **********************************************************************/
        LocalCartesian(double lat0, double lon0, double h0 );

        /**
         * Constructor that uses the provided ellipsoid.
         *
         * @param[in] earth Geocentric object for the transformation; default
         *   Geocentric::WGS84.
         *
         * Sets \e lat0 = 0, \e lon0 = 0, \e h0 = 0.
         **********************************************************************/
        LocalCartesian(Geocentric^ earth);

        /**
         * The default constructor assumes the WGS84 ellipsoid.
         *
         * Sets \e lat0 = 0, \e lon0 = 0, \e h0 = 0.
         **********************************************************************/
        LocalCartesian();

        /**
         * The destructor calls the finalizer.
         **********************************************************************/
        ~LocalCartesian()
        { this->!LocalCartesian(); }

        /**
         * Reset the origin.
         *
         * @param[in] lat0 latitude at origin (degrees).
         * @param[in] lon0 longitude at origin (degrees).
         * @param[in] h0 height above ellipsoid at origin (meters); default 0.
         *
         * \e lat0 should be in the range [&minus;90&deg;, 90&deg;].
         **********************************************************************/
        void Reset(double lat0, double lon0, double h0 );

        /**
         * Convert from geodetic to local cartesian coordinates.
         *
         * @param[in] lat latitude of point (degrees).
         * @param[in] lon longitude of point (degrees).
         * @param[in] h height of point above the ellipsoid (meters).
         * @param[out] x local cartesian coordinate (meters).
         * @param[out] y local cartesian coordinate (meters).
         * @param[out] z local cartesian coordinate (meters).
         *
         * \e lat should be in the range [&minus;90&deg;, 90&deg;].
         **********************************************************************/
        void Forward(double lat, double lon, double h,
            [System::Runtime::InteropServices::Out] double% x,
            [System::Runtime::InteropServices::Out] double% y,
            [System::Runtime::InteropServices::Out] double% z);

        /**
         * Convert from geodetic to local cartesian coordinates and return rotation
         * matrix.
         *
         * @param[in] lat latitude of point (degrees).
         * @param[in] lon longitude of point (degrees).
         * @param[in] h height of point above the ellipsoid (meters).
         * @param[out] x local cartesian coordinate (meters).
         * @param[out] y local cartesian coordinate (meters).
         * @param[out] z local cartesian coordinate (meters).
         * @param[out] M a 3 &times; 3 rotation matrix.
         *
         * \e lat should be in the range [&minus;90&deg;, 90&deg;].
         *
         * Let \e v be a unit vector located at (\e lat, \e lon, \e h).  We can
         * express \e v as \e column vectors in one of two ways
         * - in east, north, up coordinates (where the components are relative to a
         *   local coordinate system at (\e lat, \e lon, \e h)); call this
         *   representation \e v1.
         * - in \e x, \e y, \e z coordinates (where the components are relative to
         *   the local coordinate system at (\e lat0, \e lon0, \e h0)); call this
         *   representation \e v0.
         * .
         * Then we have \e v0 = \e M &sdot; \e v1.
         **********************************************************************/
        void Forward(double lat, double lon, double h,
            [System::Runtime::InteropServices::Out] double% x,
            [System::Runtime::InteropServices::Out] double% y,
            [System::Runtime::InteropServices::Out] double% z,
            [System::Runtime::InteropServices::Out] array<double,2>^% M);

        /**
         * Convert from local cartesian to geodetic coordinates.
         *
         * @param[in] x local cartesian coordinate (meters).
         * @param[in] y local cartesian coordinate (meters).
         * @param[in] z local cartesian coordinate (meters).
         * @param[out] lat latitude of point (degrees).
         * @param[out] lon longitude of point (degrees).
         * @param[out] h height of point above the ellipsoid (meters).
         *
         * The value of \e lon returned is in the range [&minus;180&deg;,
         * 180&deg;).
         **********************************************************************/
        void Reverse(double x, double y, double z,
            [System::Runtime::InteropServices::Out] double% lat,
            [System::Runtime::InteropServices::Out] double% lon,
            [System::Runtime::InteropServices::Out] double% h);

        /**
         * Convert from local cartesian to geodetic coordinates and return rotation
         * matrix.
         *
         * @param[in] x local cartesian coordinate (meters).
         * @param[in] y local cartesian coordinate (meters).
         * @param[in] z local cartesian coordinate (meters).
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
         * - in \e x, \e y, \e z coordinates (where the components are relative to
         *   the local coordinate system at (\e lat0, \e lon0, \e h0)); call this
         *   representation \e v0.
         * .
         * Then we have \e v1 = \e M<sup>T</sup> &sdot; \e v0, where \e
         * M<sup>T</sup> is the transpose of \e M.
         **********************************************************************/
        void Reverse(double x, double y, double z,
            [System::Runtime::InteropServices::Out] double% lat,
            [System::Runtime::InteropServices::Out] double% lon,
            [System::Runtime::InteropServices::Out] double% h,
            [System::Runtime::InteropServices::Out] array<double,2>^% M);

        /** \name Inspector functions
         **********************************************************************/
        ///@{
        /**
         * @return latitude of the origin (degrees).
         **********************************************************************/
        property double LatitudeOrigin { double get(); }

        /**
         * @return longitude of the origin (degrees).
         **********************************************************************/
        property double LongitudeOrigin { double get(); }

        /**
         * @return height of the origin (meters).
         **********************************************************************/
        property double HeightOrigin { double get(); }

        /**
         * @return \e a the equatorial radius of the ellipsoid (meters).  This is
         *   the value of \e a inherited from the Geocentric object used in the
         *   constructor.
         **********************************************************************/
        property double MajorRadius { double get(); }

        /**
         * @return \e f the flattening of the ellipsoid.  This is the value
         *   inherited from the Geocentric object used in the constructor.
         **********************************************************************/
        property double Flattening { double get(); }
        ///@}
    };
} // namespace NETGeographicLib
