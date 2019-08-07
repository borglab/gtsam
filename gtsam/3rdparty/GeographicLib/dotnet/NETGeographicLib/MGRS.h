#pragma once
/**
 * \file NETGeographicLib/MGRS.h
 * \brief Header for NETGeographicLib::MGRS class
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
   * \brief .NET wrapper for GeographicLib::MGRS.
   *
   * This class allows .NET applications to access GeographicLib::MGRS.
   *
   * MGRS is defined in Chapter 3 of
   * - J. W. Hager, L. L. Fry, S. S. Jacks, D. R. Hill,
   *   <a href="http://earth-info.nga.mil/GandG/publications/tm8358.1/pdf/TM8358_1.pdf">

   *   Datums, Ellipsoids, Grids, and Grid Reference Systems</a>,
   *   Defense Mapping Agency, Technical Manual TM8358.1 (1990).
   *
   * This implementation has the following properties:
   * - The conversions are closed, i.e., output from Forward is legal input for
   *   Reverse and vice versa.  Conversion in both directions preserve the
   *   UTM/UPS selection and the UTM zone.
   * - Forward followed by Reverse and vice versa is approximately the
   *   identity.  (This is affected in predictable ways by errors in
   *   determining the latitude band and by loss of precision in the MGRS
   *   coordinates.)
   * - All MGRS coordinates truncate to legal 100 km blocks.  All MGRS
   *   coordinates with a legal 100 km block prefix are legal (even though the
   *   latitude band letter may now belong to a neighboring band).
   * - The range of UTM/UPS coordinates allowed for conversion to MGRS
   *   coordinates is the maximum consistent with staying within the letter
   *   ranges of the MGRS scheme.
   * - All the transformations are implemented as static methods in the MGRS
   *   class.
   *
   * The <a href="http://www.nga.mil">NGA</a> software package
   * <a href="http://earth-info.nga.mil/GandG/geotrans/index.html">geotrans</a>
   * also provides conversions to and from MGRS.  Version 3.0 (and earlier)
   * suffers from some drawbacks:
   * - Inconsistent rules are used to determine the whether a particular MGRS
   *   coordinate is legal.  A more systematic approach is taken here.
   * - The underlying projections are not very accurately implemented.
   *
   * C# Example:
   * \include example-MGRS.cs
   * Managed C++ Example:
   * \include example-MGRS.cpp
   * Visual Basic Example:
   * \include example-MGRS.vb
   *
   **********************************************************************/
    public ref class MGRS
    {
        private:
        // Hide the constructor since all members are static.
        MGRS(void) {}
    public:

        /**
         * Convert UTM or UPS coordinate to an MGRS coordinate.
         *
         * @param[in] zone UTM zone (zero means UPS).
         * @param[in] northp hemisphere (true means north, false means south).
         * @param[in] x easting of point (meters).
         * @param[in] y northing of point (meters).
         * @param[in] prec precision relative to 100 km.
         * @param[out] mgrs MGRS string.
         * @exception GeographicErr if \e zone, \e x, or \e y is outside its
         *   allowed range.
         * @exception GeographicErr if the memory for the MGRS string can't be
         *   allocated.
         *
         * \e prec specifies the precision of the MGRS string as follows:
         * - prec = &minus;1 (min), only the grid zone is returned
         * - prec = 0 (min), 100 km
         * - prec = 1, 10 km
         * - prec = 2, 1 km
         * - prec = 3, 100 m
         * - prec = 4, 10 m
         * - prec = 5, 1 m
         * - prec = 6, 0.1 m
         * - prec = 11 (max), 1 &mu;m
         *
         * UTM eastings are allowed to be in the range [100 km, 900 km], northings
         * are allowed to be in in [0 km, 9500 km] for the northern hemisphere and
         * in [1000 km, 10000 km] for the southern hemisphere.  (However UTM
         * northings can be continued across the equator.  So the actual limits on
         * the northings are [&minus;9000 km, 9500 km] for the "northern"
         * hemisphere and [1000 km, 19500 km] for the "southern" hemisphere.)
         *
         * UPS eastings/northings are allowed to be in the range [1300 km, 2700 km]
         * in the northern hemisphere and in [800 km, 3200 km] in the southern
         * hemisphere.
         *
         * The ranges are 100 km more restrictive that for the conversion between
         * geographic coordinates and UTM and UPS given by UTMUPS.  These
         * restrictions are dictated by the allowed letters in MGRS coordinates.
         * The choice of 9500 km for the maximum northing for northern hemisphere
         * and of 1000 km as the minimum northing for southern hemisphere provide
         * at least 0.5 degree extension into standard UPS zones.  The upper ends
         * of the ranges for the UPS coordinates is dictated by requiring symmetry
         * about the meridians 0E and 90E.
         *
         * All allowed UTM and UPS coordinates may now be converted to legal MGRS
         * coordinates with the proviso that eastings and northings on the upper
         * boundaries are silently reduced by about 4 nm (4 nanometers) to place
         * them \e within the allowed range.  (This includes reducing a southern
         * hemisphere northing of 10000 km by 4 nm so that it is placed in latitude
         * band M.)  The UTM or UPS coordinates are truncated to requested
         * precision to determine the MGRS coordinate.  Thus in UTM zone 38n, the
         * square area with easting in [444 km, 445 km) and northing in [3688 km,
         * 3689 km) maps to MGRS coordinate 38SMB4488 (at \e prec = 2, 1 km),
         * Khulani Sq., Baghdad.
         *
         * The UTM/UPS selection and the UTM zone is preserved in the conversion to
         * MGRS coordinate.  Thus for \e zone > 0, the MGRS coordinate begins with
         * the zone number followed by one of [C--M] for the southern
         * hemisphere and [N--X] for the northern hemisphere.  For \e zone =
         * 0, the MGRS coordinates begins with one of [AB] for the southern
         * hemisphere and [XY] for the northern hemisphere.
         *
         * The conversion to the MGRS is exact for prec in [0, 5] except that a
         * neighboring latitude band letter may be given if the point is within 5nm
         * of a band boundary.  For prec in [6, 11], the conversion is accurate to
         * roundoff.
         *
         * If \e prec = &minus;1, then the "grid zone designation", e.g., 18T, is
         * returned.  This consists of the UTM zone number (absent for UPS) and the
         * first letter of the MGRS string which labels the latitude band for UTM
         * and the hemisphere for UPS.
         *
         * If \e x or \e y is NaN or if \e zone is UTMUPS::INVALID, the returned
         * MGRS string is "INVALID".
         *
         * Return the result via a reference argument to avoid the overhead of
         * allocating a potentially large number of small strings.  If an error is
         * thrown, then \e mgrs is unchanged.
         **********************************************************************/
        static void Forward(int zone, bool northp, double x, double y,
                            int prec,
                [System::Runtime::InteropServices::Out] System::String^% mgrs);

        /**
         * Convert UTM or UPS coordinate to an MGRS coordinate when the latitude is
         * known.
         *
         * @param[in] zone UTM zone (zero means UPS).
         * @param[in] northp hemisphere (true means north, false means south).
         * @param[in] x easting of point (meters).
         * @param[in] y northing of point (meters).
         * @param[in] lat latitude (degrees).
         * @param[in] prec precision relative to 100 km.
         * @param[out] mgrs MGRS string.
         * @exception GeographicErr if \e zone, \e x, or \e y is outside its
         *   allowed range.
         * @exception GeographicErr if \e lat is inconsistent with the given UTM
         *   coordinates.
         * @exception std::bad_alloc if the memory for \e mgrs can't be allocated.
         *
         * The latitude is ignored for \e zone = 0 (UPS); otherwise the latitude is
         * used to determine the latitude band and this is checked for consistency
         * using the same tests as Reverse.
         **********************************************************************/
        static void Forward(int zone, bool northp, double x, double y, double lat,
                int prec,
                [System::Runtime::InteropServices::Out] System::String^% mgrs);

        /**
         * Convert a MGRS coordinate to UTM or UPS coordinates.
         *
         * @param[in] mgrs MGRS string.
         * @param[out] zone UTM zone (zero means UPS).
         * @param[out] northp hemisphere (true means north, false means south).
         * @param[out] x easting of point (meters).
         * @param[out] y northing of point (meters).
         * @param[out] prec precision relative to 100 km.
         * @param[in] centerp if true (default), return center of the MGRS square,
         *   else return SW (lower left) corner.
         * @exception GeographicErr if \e mgrs is illegal.
         *
         * All conversions from MGRS to UTM/UPS are permitted provided the MGRS
         * coordinate is a possible result of a conversion in the other direction.
         * (The leading 0 may be dropped from an input MGRS coordinate for UTM
         * zones 1--9.)  In addition, MGRS coordinates with a neighboring
         * latitude band letter are permitted provided that some portion of the
         * 100 km block is within the given latitude band.  Thus
         *   - 38VLS and 38WLS are allowed (latitude 64N intersects the square
         *     38[VW]LS); but 38VMS is not permitted (all of 38VMS is north of 64N)
         *   - 38MPE and 38NPF are permitted (they straddle the equator); but 38NPE
         *     and 38MPF are not permitted (the equator does not intersect either
         *     block).
         *   - Similarly ZAB and YZB are permitted (they straddle the prime
         *     meridian); but YAB and ZZB are not (the prime meridian does not
         *     intersect either block).
         *
         * The UTM/UPS selection and the UTM zone is preserved in the conversion
         * from MGRS coordinate.  The conversion is exact for prec in [0, 5].  With
         * centerp = true the conversion from MGRS to geographic and back is
         * stable.  This is not assured if \e centerp = false.
         *
         * If a "grid zone designation" (for example, 18T or A) is given, then some
         * suitable (but essentially arbitrary) point within that grid zone is
         * returned.  The main utility of the conversion is to allow \e zone and \e
         * northp to be determined.  In this case, the \e centerp parameter is
         * ignored and \e prec is set to &minus;1.
         *
         * If the first 3 characters of \e mgrs are "INV", then \e x and \e y are
         * set to NaN, \e zone is set to UTMUPS::INVALID, and \e prec is set to
         * &minus;2.
         *
         * If an exception is thrown, then the arguments are unchanged.
         **********************************************************************/
        static void Reverse(System::String^ mgrs,
                        [System::Runtime::InteropServices::Out] int% zone,
                        [System::Runtime::InteropServices::Out] bool% northp,
                        [System::Runtime::InteropServices::Out] double% x,
                        [System::Runtime::InteropServices::Out] double% y,
                        [System::Runtime::InteropServices::Out] int% prec,
                        bool centerp );

        /** \name Inspector functions
         **********************************************************************/
        ///@{
        /**
         * @return \e a the equatorial radius of the WGS84 ellipsoid (meters).
         *
         * (The WGS84 value is returned because the UTM and UPS projections are
         * based on this ellipsoid.)
         **********************************************************************/
        static double MajorRadius();

        /**
         * @return \e f the flattening of the WGS84 ellipsoid.
         *
         * (The WGS84 value is returned because the UTM and UPS projections are
         * based on this ellipsoid.)
         **********************************************************************/
        static double Flattening();
        ///@}
    };
} // namespace NETGeographicLib
