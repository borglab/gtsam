/**
 * \file Geohash.hpp
 * \brief Header for GeographicLib::Geohash class
 *
 * Copyright (c) Charles Karney (2012-2017) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_GEOHASH_HPP)
#define GEOGRAPHICLIB_GEOHASH_HPP 1

#include <GeographicLib/Constants.hpp>

#if defined(_MSC_VER)
// Squelch warnings about dll vs string
#  pragma warning (push)
#  pragma warning (disable: 4251)
#endif

namespace GeographicLib {

  /**
   * \brief Conversions for geohashes
   *
   * Geohashes are described in
   * - https://en.wikipedia.org/wiki/Geohash
   * - http://geohash.org/
   * .
   * They provide a compact string representation of a particular geographic
   * location (expressed as latitude and longitude), with the property that if
   * trailing characters are dropped from the string the geographic location
   * remains nearby.  The classes Georef and GARS implement similar compact
   * representations.
   *
   * Example of use:
   * \include example-Geohash.cpp
   **********************************************************************/

  class GEOGRAPHICLIB_EXPORT Geohash {
  private:
    typedef Math::real real;
    static const int maxlen_ = 18;
    static const unsigned long long mask_ = 1ULL << 45;
    static const char* const lcdigits_;
    static const char* const ucdigits_;
    Geohash();                     // Disable constructor

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
     * Internally, \e len is first put in the range [0, 18].  (\e len = 18
     * provides approximately 1&mu;m precision.)
     *
     * If \e lat or \e lon is NaN, the returned geohash is "invalid".
     **********************************************************************/
    static void Forward(real lat, real lon, int len, std::string& geohash);

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
     * Only the first 18 characters for \e geohash are considered.  (18
     * characters provides approximately 1&mu;m precision.)  The case of the
     * letters in \e geohash is ignored.
     *
     * If the first 3 characters of \e geohash are "inv", then \e lat and \e
     * lon are set to NaN and \e len is unchanged.  ("nan" is treated
     * similarly.)
     **********************************************************************/
    static void Reverse(const std::string& geohash, real& lat, real& lon,
                        int& len, bool centerp = true);

    /**
     * The latitude resolution of a geohash.
     *
     * @param[in] len the length of the geohash.
     * @return the latitude resolution (degrees).
     *
     * Internally, \e len is first put in the range [0, 18].
     **********************************************************************/
    static Math::real LatitudeResolution(int len) {
      using std::ldexp;
      len = (std::max)(0, (std::min)(int(maxlen_), len));
      return ldexp(real(180), -(5 * len / 2));
    }

    /**
     * The longitude resolution of a geohash.
     *
     * @param[in] len the length of the geohash.
     * @return the longitude resolution (degrees).
     *
     * Internally, \e len is first put in the range [0, 18].
     **********************************************************************/
    static Math::real LongitudeResolution(int len) {
      using std::ldexp;
      len = (std::max)(0, (std::min)(int(maxlen_), len));
      return ldexp(real(360), -(5 * len - 5 * len / 2));
    }

    /**
     * The geohash length required to meet a given geographic resolution.
     *
     * @param[in] res the minimum of resolution in latitude and longitude
     *   (degrees).
     * @return geohash length.
     *
     * The returned length is in the range [0, 18].
     **********************************************************************/
    static int GeohashLength(real res) {
      using std::abs; res = abs(res);
      for (int len = 0; len < maxlen_; ++len)
        if (LongitudeResolution(len) <= res)
          return len;
      return maxlen_;
    }

    /**
     * The geohash length required to meet a given geographic resolution.
     *
     * @param[in] latres the resolution in latitude (degrees).
     * @param[in] lonres the resolution in longitude (degrees).
     * @return geohash length.
     *
     * The returned length is in the range [0, 18].
     **********************************************************************/
    static int GeohashLength(real latres, real lonres) {
      using std::abs;
      latres = abs(latres);
      lonres = abs(lonres);
      for (int len = 0; len < maxlen_; ++len)
        if (LatitudeResolution(len) <= latres &&
            LongitudeResolution(len) <= lonres)
          return len;
      return maxlen_;
    }

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
    static int DecimalPrecision(int len) {
      using std::floor; using std::log;
      return -int(floor(log(LatitudeResolution(len))/log(Math::real(10))));
    }

  };

} // namespace GeographicLib

#if defined(_MSC_VER)
#  pragma warning (pop)
#endif

#endif  // GEOGRAPHICLIB_GEOHASH_HPP
