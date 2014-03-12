/**
 * \file Geohash.cpp
 * \brief Implementation for GeographicLib::Geohash class
 *
 * Copyright (c) Charles Karney (2012) <charles@karney.com> and licensed under
 * the MIT/X11 License.  For more information, see
 * http://geographiclib.sourceforge.net/
 **********************************************************************/

#include <GeographicLib/Geohash.hpp>
#include <GeographicLib/Utility.hpp>

namespace GeographicLib {

  using namespace std;

  const int Geohash::decprec_[] = {-2, -1, 0, 0, 1, 2, 3, 3, 4, 5,
                                   6, 6, 7, 8, 9, 9, 10, 11, 12};
  const Math::real Geohash::loneps_ = 180 * std::pow(0.5, 45);
  const Math::real Geohash::lateps_ = 90 * std::pow(0.5, 45);
  const Math::real Geohash::shift_ = std::pow(2.0, 45);
  const string Geohash::lcdigits_ = "0123456789bcdefghjkmnpqrstuvwxyz";
  const string Geohash::ucdigits_ = "0123456789BCDEFGHJKMNPQRSTUVWXYZ";

  void Geohash::Forward(real lat, real lon, int len, std::string& geohash) {
    if (abs(lat) > 90)
      throw GeographicErr("Latitude " + Utility::str(lat)
                          + "d not in [-90d, 90d]");
    if (lon < -540 || lon >= 540)
      throw GeographicErr("Longitude " + Utility::str(lon)
                          + "d not in [-540d, 540d)");
    if (Math::isnan(lat) || Math::isnan(lon)) {
      geohash = "nan";
      return;
    }
    if (lat == 90) lat -= lateps_ / 2;
    lon = Math::AngNormalize(lon); // lon in [-180,180)
    // lon/loneps_ in [-2^45,2^45); lon/eps + shift_ in [0,2^46)
    // similarly for lat
    len = max(0, min(int(maxlen_), len));
    unsigned long long
      ulon = (unsigned long long)(floor(lon/loneps_) + shift_),
      ulat = (unsigned long long)(floor(lat/lateps_) + shift_);
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
    len = min(int(maxlen_), int(geohash.length()));
    if (len >= 3 &&
        toupper(geohash[0]) == 'N' &&
        toupper(geohash[1]) == 'A' &&
        toupper(geohash[2]) == 'N') {
      lat = lon = Math::NaN<real>();
      return;
    }
    unsigned long long ulon = 0, ulat = 0;
    for (unsigned k = 0, j = 0; k < unsigned(len); ++k) {
      int byte = Utility::lookup(ucdigits_, geohash[k]);
      if (byte < 0)
        throw GeographicErr("Illegal character in geohash " + geohash);
      for (unsigned i = 0, m = 16; i < 5; ++i, m >>= 1) {
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
    int s = 5 * (maxlen_ - len);
    ulon <<=     (s / 2);
    ulat <<= s - (s / 2);
    lon = ulon * loneps_ - 180;
    lat = ulat * lateps_ - 90;
  }

} // namespace GeographicLib
