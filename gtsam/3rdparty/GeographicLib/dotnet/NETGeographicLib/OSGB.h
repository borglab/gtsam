/**
 * \file NETGeographicLib/OSGB.h
 * \brief Header for NETGeographicLib::OSGB class
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
   * \brief .NET wrapper for GeographicLib::OSGB.
   *
   * This class allows .NET applications to access GeographicLib::OSGB.
   *
   * The class implements the coordinate system used by the Ordnance Survey for
   * maps of Great Britain and conversions to the grid reference system.
   *
   * See
   * - <a href="http://www.ordnancesurvey.co.uk/docs/support/guide-coordinate-systems-great-britain.pdf">
   *   A guide to coordinate systems in Great Britain</a>
   * - <a href="http://www.ordnancesurvey.co.uk/docs/support/national-grid.pdf">
   *   Guide to the National Grid</a>
   *
   * \b WARNING: the latitudes and longitudes for the Ordnance Survey grid
   * system do not use the WGS84 datum.  Do not use the values returned by this
   * class in the UTMUPS, MGRS, or Geoid classes without first converting the
   * datum (and vice versa).
   *
   * C# Example:
   * \include example-OSGB.cs
   * Managed C++ Example:
   * \include example-OSGB.cpp
   * Visual Basic Example:
   * \include example-OSGB.vb
   **********************************************************************/
    public ref class OSGB
    {
        private:
        // hide the constructor since all member are static
        OSGB(void) {}
    public:

        /**
         * Forward projection, from geographic to OSGB coordinates.
         *
         * @param[in] lat latitude of point (degrees).
         * @param[in] lon longitude of point (degrees).
         * @param[out] x easting of point (meters).
         * @param[out] y northing of point (meters).
         * @param[out] gamma meridian convergence at point (degrees).
         * @param[out] k scale of projection at point.
         *
         * \e lat should be in the range [&minus;90&deg;, 90&deg;].
         **********************************************************************/
        static void Forward(double lat, double lon,
                    [System::Runtime::InteropServices::Out] double% x,
                    [System::Runtime::InteropServices::Out] double% y,
                    [System::Runtime::InteropServices::Out] double% gamma,
                    [System::Runtime::InteropServices::Out] double% k);

        /**
         * Reverse projection, from OSGB coordinates to geographic.
         *
         * @param[in] x easting of point (meters).
         * @param[in] y northing of point (meters).
         * @param[out] lat latitude of point (degrees).
         * @param[out] lon longitude of point (degrees).
         * @param[out] gamma meridian convergence at point (degrees).
         * @param[out] k scale of projection at point.
         *
         * The value of \e lon returned is in the range [&minus;180&deg;,
         * 180&deg;).
         **********************************************************************/

        static void Reverse(double x, double y,
                    [System::Runtime::InteropServices::Out] double% lat,
                    [System::Runtime::InteropServices::Out] double% lon,
                    [System::Runtime::InteropServices::Out] double% gamma,
                    [System::Runtime::InteropServices::Out] double% k);

        /**
         * OSGB::Forward without returning the convergence and scale.
         **********************************************************************/
        static void Forward(double lat, double lon,
            [System::Runtime::InteropServices::Out] double% x,
            [System::Runtime::InteropServices::Out] double% y);

        /**
         * OSGB::Reverse without returning the convergence and scale.
         **********************************************************************/
        static void Reverse(double x, double y,
            [System::Runtime::InteropServices::Out] double% lat,
            [System::Runtime::InteropServices::Out] double% lon);

        /**
         * Convert OSGB coordinates to a grid reference.
         *
         * @param[in] x easting of point (meters).
         * @param[in] y northing of point (meters).
         * @param[in] prec precision relative to 100 km.
         * @param[out] gridref National Grid reference.
         * @exception GeographicErr if \e prec, \e x, or \e y is outside its
         *   allowed range.
         * @exception std::bad_alloc if the memory for \e gridref can't be
         *   allocatied.
         *
         * \e prec specifies the precision of the grid reference string as follows:
         * - prec = 0 (min), 100km
         * - prec = 1, 10km
         * - prec = 2, 1km
         * - prec = 3, 100m
         * - prec = 4, 10m
         * - prec = 5, 1m
         * - prec = 6, 0.1m
         * - prec = 11 (max), 1&mu;m
         *
         * The easting must be in the range [&minus;1000 km, 1500 km) and the
         * northing must be in the range [&minus;500 km, 2000 km).  These bounds
         * are consistent with rules for the letter designations for the grid
         * system.
         *
         * If \e x or \e y is NaN, the returned grid reference is "INVALID".
         **********************************************************************/
        static void GridReference(double x, double y, int prec,
            [System::Runtime::InteropServices::Out] System::String^% gridref);

        /**
         * Convert OSGB coordinates to a grid reference.
         *
         * @param[in] gridref National Grid reference.
         * @param[out] x easting of point (meters).
         * @param[out] y northing of point (meters).
         * @param[out] prec precision relative to 100 km.
         * @param[in] centerp if true (default), return center of the grid square,
         *   else return SW (lower left) corner.
         * @exception GeographicErr if \e gridref is illegal.
         *
         * The grid reference must be of the form: two letters (not including I)
         * followed by an even number of digits (up to 22).
         *
         * If the first 2 characters of \e gridref are "IN", then \e x and \e y are
         * set to NaN and \e prec is set to &minus;2.
         **********************************************************************/
        static void GridReference(System::String^ gridref,
                [System::Runtime::InteropServices::Out] double% x,
                [System::Runtime::InteropServices::Out] double% y,
                [System::Runtime::InteropServices::Out] int% prec,
                bool centerp );

        /** \name Inspector functions
         **********************************************************************/
        ///@{
        /**
         * @return \e a the equatorial radius of the Airy 1830 ellipsoid (meters).
         *
         * This is 20923713 ft converted to meters using the rule 1 ft =
         * 10<sup>9.48401603&minus;10</sup> m.  (The Airy 1830 value is returned
         * because the OSGB projection is based on this ellipsoid.)
         **********************************************************************/
        static double MajorRadius();

        /**
         * @return \e f the inverse flattening of the Airy 1830 ellipsoid.
         *
         * For the Airy 1830 ellipsoid, \e a = 20923713 ft and \e b = 20853810 ft;
         * thus the flattening = (20923713 &minus; 20853810)/20923713 =
         * 7767/2324857 = 1/299.32496459...  (The Airy 1830 value is returned
         * because the OSGB projection is based on this ellipsoid.)
         **********************************************************************/
        static double Flattening();

        /**
         * @return \e k0 central scale for the OSGB projection (0.9996012717).
         **********************************************************************/
        static double CentralScale();

        /**
         * @return latitude of the origin for the OSGB projection (49 degrees).
         **********************************************************************/
        static double OriginLatitude();

        /**
         * @return longitude of the origin for the OSGB projection (&minus;2
         *   degrees).
         **********************************************************************/
        static double OriginLongitude();

        /**
         * @return false northing the OSGB projection (&minus;100000 meters).
         **********************************************************************/
        static double FalseNorthing();

        /**
         * @return false easting the OSGB projection (400000 meters).
         **********************************************************************/
        static double FalseEasting();
        ///@}
    };
} //
