/**
 * \file GARS.hpp
 * \brief Header for GeographicLib::GARS class
 *
 * Copyright (c) Charles Karney (2015-2017) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_GARS_HPP)
#define GEOGRAPHICLIB_GARS_HPP 1

#include <GeographicLib/Constants.hpp>

#if defined(_MSC_VER)
// Squelch warnings about dll vs string
#  pragma warning (push)
#  pragma warning (disable: 4251)
#endif

namespace GeographicLib {

  /**
   * \brief Conversions for the Global Area Reference System (GARS)
   *
   * The Global Area Reference System is described in
   * - https://en.wikipedia.org/wiki/Global_Area_Reference_System
   * - http://earth-info.nga.mil/GandG/coordsys/grids/gars.html
   * .
   * It provides a compact string representation of a geographic area
   * (expressed as latitude and longitude).  The classes Georef and Geohash
   * implement similar compact representations.
   *
   * Example of use:
   * \include example-GARS.cpp
   **********************************************************************/

  class GEOGRAPHICLIB_EXPORT GARS {
  private:
    typedef Math::real real;
    static const char* const digits_;
    static const char* const letters_;
    enum {
      lonorig_ = -180,          // Origin for longitude
      latorig_ = -90,           // Origin for latitude
      baselon_ = 10,            // Base for longitude tiles
      baselat_ = 24,            // Base for latitude tiles
      lonlen_ = 3,
      latlen_ = 2,
      baselen_ = lonlen_ + latlen_,
      mult1_ = 2,               // base precision = 1/2 degree
      mult2_ = 2,               // 6th char gives 2x more precision
      mult3_ = 3,               // 7th char gives 3x more precision
      m_ = mult1_ * mult2_ * mult3_,
      maxprec_ = 2,
      maxlen_ = baselen_ + maxprec_,
    };
    GARS();                     // Disable constructor

  public:

    /**
     * Convert from geographic coordinates to GARS.
     *
     * @param[in] lat latitude of point (degrees).
     * @param[in] lon longitude of point (degrees).
     * @param[in] prec the precision of the resulting GARS.
     * @param[out] gars the GARS string.
     * @exception GeographicErr if \e lat is not in [&minus;90&deg;,
     *   90&deg;].
     * @exception std::bad_alloc if memory for \e gars can't be allocated.
     *
     * \e prec specifies the precision of \e gars as follows:
     * - \e prec = 0 (min), 30' precision, e.g., 006AG;
     * - \e prec = 1, 15' precision, e.g., 006AG3;
     * - \e prec = 2 (max), 5' precision, e.g., 006AG39.
     *
     * If \e lat or \e lon is NaN, then \e gars is set to "INVALID".
     **********************************************************************/
    static void Forward(real lat, real lon, int prec, std::string& gars);

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
    static void Reverse(const std::string& gars, real& lat, real& lon,
                        int& prec, bool centerp = true);

    /**
     * The angular resolution of a GARS.
     *
     * @param[in] prec the precision of the GARS.
     * @return the latitude-longitude resolution (degrees).
     *
     * Internally, \e prec is first put in the range [0, 2].
     **********************************************************************/
    static Math::real Resolution(int prec) {
      return 1/real(prec <= 0 ? mult1_ : (prec == 1 ? mult1_ * mult2_ :
                                          mult1_ * mult2_ * mult3_));
    }

    /**
     * The GARS precision required to meet a given geographic resolution.
     *
     * @param[in] res the minimum of resolution in latitude and longitude
     *   (degrees).
     * @return GARS precision.
     *
     * The returned length is in the range [0, 2].
     **********************************************************************/
    static int Precision(real res) {
      using std::abs; res = abs(res);
      for (int prec = 0; prec < maxprec_; ++prec)
        if (Resolution(prec) <= res)
          return prec;
      return maxprec_;
    }

  };

} // namespace GeographicLib

#if defined(_MSC_VER)
#  pragma warning (pop)
#endif

#endif  // GEOGRAPHICLIB_GARS_HPP
