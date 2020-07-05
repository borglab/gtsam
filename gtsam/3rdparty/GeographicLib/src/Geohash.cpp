/**
 * \file Geohash.cpp
 * \brief Implementation for GeographicLib::Geohash class
 *
 * Copyright (c) Charles Karney (2012-2017) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <GeographicLib/Geohash.hpp>
#include <GeographicLib/Utility.hpp>

namespace GeographicLib {

  using namespace std;

  const char* const Geohash::lcdigits_ = "0123456789bcdefghjkmnpqrstuvwxyz";
  const char* const Geohash::ucdigits_ = "0123456789BCDEFGHJKMNPQRSTUVWXYZ";

  void Geohash::Forward(real lat, real lon, int len, std::string& geohash) {
    static const real shift = ldexp(real(1), 45);
    static const real loneps = 180 / shift;
    static const real lateps =  90 / shift;
    if (abs(lat) > 90)
      throw GeographicErr("Latitude " + Utility::str(lat)
                          + "d not in [-90d, 90d]");
    if (Math::isnan(lat) || Math::isnan(lon)) {
      geohash = "invalid";
      return;
    }
    if (lat == 90) lat -= lateps / 2;
    lon = Math::AngNormalize(lon); // lon in [-180,180)
    // lon/loneps in [-2^45,2^45); lon/loneps + shift in [0,2^46)
    // similarly for lat
    len = max(0, min(int(maxlen_), len));
    unsigned long long
      ulon = (unsigned long long)(floor(lon/loneps) + shift),
      ulat = (unsigned long long)(floor(lat/lateps) + shift);
    char geohash1[maxlen_];
    unsigned byte = 0;
    for (unsigned i = 0; i < 5 * unsigned(len);) {
      if ((i & 1) == 0) {
        byte = (byte << 1) + unsigned((ulon & mask_) != 0);
        ulon <<= 1;
      } else {
        byte = (byte << 1) + unsigned((ulat & mask_) != 0);
        ulat <<= 1;
      }
      ++i;
      if (i % 5 == 0) {
        geohash1[(i/5)-1] = lcdigits_[byte];
        byte = 0;
      }
    }
    geohash.resize(len);
    copy(geohash1, geohash1 + len, geohash.begin());
  }

  void Geohash::Reverse(const std::string& geohash, real& lat, real& lon,
                        int& len, bool centerp) {
    static const real shift = ldexp(real(1), 45);
    static const real loneps = 180 / shift;
    static const real lateps =  90 / shift;
    int len1 = min(int(maxlen_), int(geohash.length()));
    if (len1 >= 3 &&
        ((toupper(geohash[0]) == 'I' &&
          toupper(geohash[1]) == 'N' &&
          toupper(geohash[2]) == 'V') ||
         // Check A first because it is not in a standard geohash
         (toupper(geohash[1]) == 'A' &&
          toupper(geohash[0]) == 'N' &&
          toupper(geohash[2]) == 'N'))) {
      lat = lon = Math::NaN();
      return;
    }
    unsigned long long ulon = 0, ulat = 0;
    for (unsigned k = 0, j = 0; k < unsigned(len1); ++k) {
      int byte = Utility::lookup(ucdigits_, geohash[k]);
      if (byte < 0)
        throw GeographicErr("Illegal character in geohash " + geohash);
      for (unsigned m = 16; m; m >>= 1) {
        if (j == 0)
          ulon = (ulon << 1) + unsigned((byte & m) != 0);
        else
          ulat = (ulat << 1) + unsigned((byte & m) != 0);
        j ^= 1;
      }
    }
    ulon <<= 1; ulat <<= 1;
    if (centerp) {
      ulon += 1;
      ulat += 1;
    }
    int s = 5 * (maxlen_ - len1);
    ulon <<=     (s / 2);
    ulat <<= s - (s / 2);
    lon = ulon * loneps - 180;
    lat = ulat * lateps - 90;
    len = len1;
  }

} // namespace GeographicLib
