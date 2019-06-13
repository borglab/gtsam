#pragma once
/**
 * \file NETGeographicLib/Geohash.h
 * \brief Header for NETGeographicLib::Geohash class
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
   * \brief .NET wrapper for GeographicLib::Geohash.
   *
   * Geohashes are described in
   * - https://en.wikipedia.org/wiki/Geohash
   * - http://geohash.org/ (this link is broken as of 2012-12-11)
   * .
   * They provide a compact string representation of a particular geographic
   * location (expressed as latitude and longitude), with the property that if
   * trailing characters are dropped from the string the geographic location
   * remains nearby.
   *
   * C# Example:
   * \include example-Geohash.cs
   * Managed C++ Example:
   * \include example-Geohash.cpp
   * Visual Basic Example:
   * \include example-Geohash.vb
   **********************************************************************/
    public ref class Geohash
    {
        private:
        // hide the constructor since all members of this class are static.
        Geohash() {}
    public:

        /**
         * Convert from geographic coordinates to a geohash.
         *
         * @param[in] lat latitude of point (degrees).
         * @param[in] lon longitude of point (degrees).
         * @param[in] len the length of the resulting geohash.
         * @param[out] geohash the geohash.
         * @exception GeographicErr if \e lat is not in [&minus;90&deg;,
         *   90&deg;].
         * @exception std::bad_alloc if memory for \e geohash can't be allocated.
         *
         * Internally, \e len is first put in the range [0, 18].
         *
         * If \e lat or \e lon is NaN, the returned geohash is "nan".
         **********************************************************************/
        static void Forward(double lat, double lon, int len,
            [System::Runtime::InteropServices::Out] System::String^% geohash);

        /**
         * Convert from a geohash to geographic coordinates.
         *
         * @param[in] geohash the geohash.
         * @param[out] lat latitude of point (degrees).
         * @param[out] lon longitude of point (degrees).
         * @param[out] len the length of the geohash.
         * @param[in] centerp if true (the default) return the center of the
         *   geohash location, otherwise return the south-west corner.
         * @exception GeographicErr if \e geohash contains illegal characters.
         *
         * Only the first 18 characters for \e geohash are considered.  The case of
         * the letters in \e geohash is ignored.
         *
         * If the first three characters in \e geohash are "nan", then \e lat and
         * \e lon are set to NaN.
         **********************************************************************/
        static void Reverse(System::String^ geohash,
            [System::Runtime::InteropServices::Out] double% lat,
            [System::Runtime::InteropServices::Out] double% lon,
            [System::Runtime::InteropServices::Out] int% len,
            bool centerp);

        /**
         * The latitude resolution of a geohash.
         *
         * @param[in] len the length of the geohash.
         * @return the latitude resolution (degrees).
         *
         * Internally, \e len is first put in the range [0, 18].
         **********************************************************************/
        static double LatitudeResolution(int len);

        /**
         * The longitude resolution of a geohash.
         *
         * @param[in] len the length of the geohash.
         * @return the longitude resolution (degrees).
         *
         * Internally, \e len is first put in the range [0, 18].
         **********************************************************************/
        static double LongitudeResolution(int len);

        /**
         * The geohash length required to meet a given geographic resolution.
         *
         * @param[in] res the minimum of resolution in latitude and longitude
         *   (degrees).
         * @return geohash length.
         *
         * The returned length is in the range [0, 18].
         **********************************************************************/
        static int GeohashLength(double res);

        /**
         * The geohash length required to meet a given geographic resolution.
         *
         * @param[in] latres the resolution in latitude (degrees).
         * @param[in] lonres the resolution in longitude (degrees).
         * @return geohash length.
         *
         * The returned length is in the range [0, 18].
         **********************************************************************/
        static int GeohashLength(double latres, double lonres);

        /**
         * The decimal geographic precision required to match a given geohash
         * length.  This is the number of digits needed after decimal point in a
         * decimal degrees representation.
         *
         * @param[in] len the length of the geohash.
         * @return the decimal precision (may be negative).
         *
         * Internally, \e len is first put in the range [0, 18].  The returned
         * decimal precision is in the range [&minus;2, 12].
         **********************************************************************/
        static int DecimalPrecision(int len);
    };
} // namespace NETGeographicLib
