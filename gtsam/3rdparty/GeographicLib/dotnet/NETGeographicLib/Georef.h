/**
 * \file NETGeographicLib/Georef.h
 * \brief Header for NETGeographicLib::Georef class
 *
 * NETGeographicLib is copyright (c) Scott Heiman (2013-2015)
 * GeographicLib is Copyright (c) Charles Karney (2010-2012)
 * <charles@karney.com> and licensed under the MIT/X11 License.
 * For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/
#pragma once

namespace NETGeographicLib
{
  /**
   * \brief .NET wrapper for GeographicLib::Georef.
   *
   * The World Geographic Reference System is described in
   * - https://en.wikipedia.org/wiki/Georef
   * - http://earth-info.nga.mil/GandG/coordsys/grids/georef.pdf
   * .
   * It provides a compact string representation of a geographic area
   * (expressed as latitude and longitude).  The classes GARS and Geohash
   * implement similar compact representations.
   *
   * C# Example:
   * \include example-Georef.cs
   * Managed C++ Example:
   * \include example-Georef.cpp
   * Visual Basic Example:
   * \include example-Georef.vb
   **********************************************************************/
  public ref class Georef
  {
  private:
      // hide the constructor since all members of this class are static.
      Georef() {}

  public:
    /**
     * Convert from geographic coordinates to georef.
     *
     * @param[in] lat latitude of point (degrees).
     * @param[in] lon longitude of point (degrees).
     * @param[in] prec the precision of the resulting georef.
     * @param[out] georef the georef string.
     * @exception GeographicErr if \e lat is not in [&minus;90&deg;,
     *   90&deg;] or if memory for \e georef can't be allocated.
     *
     * \e prec specifies the precision of \e georef as follows:
     * - \e prec = &minus;1 (min), 15&deg;
     * - \e prec = 0, 1&deg;
     * - \e prec = 1, converted to \e prec = 2
     * - \e prec = 2, 1'
     * - \e prec = 3, 0.1'
     * - \e prec = 4, 0.01'
     * - \e prec = 5, 0.001'
     * - &hellip;
     * - \e prec = 11 (max), 10<sup>&minus;9</sup>'
     *
     * If \e lat or \e lon is NaN, then \e georef is set to "INVALID".
     **********************************************************************/
    static void Forward(double lat, double lon, int prec,
        [System::Runtime::InteropServices::Out] System::String^% georef);

    /**
     * Convert from Georef to geographic coordinates.
     *
     * @param[in] georef the Georef.
     * @param[out] lat latitude of point (degrees).
     * @param[out] lon longitude of point (degrees).
     * @param[out] prec the precision of \e georef.
     * @param[in] centerp if true (the default) return the center
     *   \e georef, otherwise return the south-west corner.
     * @exception GeographicErr if \e georef is illegal.
     *
     * The case of the letters in \e georef is ignored.  \e prec is in the
     * range [&minus;1, 11] and gives the precision of \e georef as follows:
     * - \e prec = &minus;1 (min), 15&deg;
     * - \e prec = 0, 1&deg;
     * - \e prec = 1, not returned
     * - \e prec = 2, 1'
     * - \e prec = 3, 0.1'
     * - \e prec = 4, 0.01'
     * - \e prec = 5, 0.001'
     * - &hellip;
     * - \e prec = 11 (max), 10<sup>&minus;9</sup>'
     *
     * If the first 3 characters of \e georef are "INV", then \e lat and \e lon
     * are set to NaN and \e prec is unchanged.
     **********************************************************************/
    static void Reverse( System::String^ georef,
        [System::Runtime::InteropServices::Out] double% lat,
        [System::Runtime::InteropServices::Out] double% lon,
        [System::Runtime::InteropServices::Out] int% prec,
        bool centerp );

    /**
     * The angular resolution of a Georef.
     *
     * @param[in] prec the precision of the Georef.
     * @return the latitude-longitude resolution (degrees).
     *
     * Internally, \e prec is first put in the range [&minus;1, 11].
     **********************************************************************/
    static double Resolution(int prec);

    /**
     * The Georef precision required to meet a given geographic resolution.
     *
     * @param[in] res the minimum of resolution in latitude and longitude
     *   (degrees).
     * @return Georef precision.
     *
     * The returned length is in the range [0, 11].
     **********************************************************************/
    static int Precision(double res);
  };
}
