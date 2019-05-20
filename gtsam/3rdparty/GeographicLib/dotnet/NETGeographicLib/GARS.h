/**
 * \file NETGeographicLib/GARS.h
 * \brief Header for NETGeographicLib::GARS class
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
   * \brief .NET Wrapper for GeographicLib::GARS
   *
   * This class allows .NET applications to access GeographicLib::GARS.
   *
   * The Global Area Reference System is described in
   * - https://en.wikipedia.org/wiki/Global_Area_Reference_System
   * - http://earth-info.nga.mil/GandG/coordsys/grids/gars.html
   * .
   * It provides a compact string representation of a geographic area
   * (expressed as latitude and longitude).  The classes Georef and Geohash
   * implement similar compact representations.
   *
   * C# Example:
   * \include example-GARS.cs
   * Managed C++ Example:
   * \include example-GARS.cpp
   * Visual Basic Example:
   * \include example-GARS.vb
   **********************************************************************/
  public ref class GARS
  {
    // all memebers of this class are static so the constructor is hidden.
      GARS() {}

  public:

    /**
     * Convert from geographic coordinates to GARS.
     *
     * @param[in] lat latitude of point (degrees).
     * @param[in] lon longitude of point (degrees).
     * @param[in] prec the precision of the resulting GARS.
     * @param[out] gars the GARS string.
     * @exception GeographicErr if \e lat is not in [&minus;90&deg;,
     *   90&deg;] or if memory for \e gars can't be allocated.
     *
     * \e prec specifies the precision of \e gars as follows:
     * - \e prec = 0 (min), 30' precision, e.g., 006AG;
     * - \e prec = 1, 15' precision, e.g., 006AG3;
     * - \e prec = 2 (max), 5' precision, e.g., 006AG39.
     *
     * If \e lat or \e lon is NaN, then \e gars is set to "INVALID".
     **********************************************************************/
    static void Forward(double lat, double lon, int prec,
        [System::Runtime::InteropServices::Out] System::String^% gars);

    /**
     * Convert from GARS to geographic coordinates.
     *
     * @param[in] gars the GARS.
     * @param[out] lat latitude of point (degrees).
     * @param[out] lon longitude of point (degrees).
     * @param[out] prec the precision of \e gars.
     * @param[in] centerp if true (the default) return the center of the
     *   \e gars, otherwise return the south-west corner.
     * @exception GeographicErr if \e gars is illegal.
     *
     * The case of the letters in \e gars is ignored.  \e prec is in the range
     * [0, 2] and gives the precision of \e gars as follows:
     * - \e prec = 0 (min), 30' precision, e.g., 006AG;
     * - \e prec = 1, 15' precision, e.g., 006AG3;
     * - \e prec = 2 (max), 5' precision, e.g., 006AG39.
     *
     * If the first 3 characters of \e gars are "INV", then \e lat and \e lon
     * are set to NaN and \e prec is unchanged.
     **********************************************************************/
    static void Reverse( System::String^ gars,
        [System::Runtime::InteropServices::Out] double% lat,
        [System::Runtime::InteropServices::Out] double% lon,
        [System::Runtime::InteropServices::Out] int% prec,
        bool centerp);

    /**
     * The angular resolution of a GARS.
     *
     * @param[in] prec the precision of the GARS.
     * @return the latitude-longitude resolution (degrees).
     *
     * Internally, \e prec is first put in the range [0, 2].
     **********************************************************************/
    static double Resolution(int prec);

    /**
     * The GARS precision required to meet a given geographic resolution.
     *
     * @param[in] res the minimum of resolution in latitude and longitude
     *   (degrees).
     * @return GARS precision.
     *
     * The returned length is in the range [0, 2].
     **********************************************************************/
    static int Precision(double res);
  };
}  // namespace NETGeographicLib
