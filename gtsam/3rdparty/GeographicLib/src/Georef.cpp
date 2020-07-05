/**
 * \file Georef.cpp
 * \brief Implementation for GeographicLib::Georef class
 *
 * Copyright (c) Charles Karney (2015-2017) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <GeographicLib/Georef.hpp>
#include <GeographicLib/Utility.hpp>

namespace GeographicLib {

  using namespace std;

  const char* const Georef::digits_ = "0123456789";
  const char* const Georef::lontile_ = "ABCDEFGHJKLMNPQRSTUVWXYZ";
  const char* const Georef::lattile_ = "ABCDEFGHJKLM";
  const char* const Georef::degrees_ = "ABCDEFGHJKLMNPQ";

  void Georef::Forward(real lat, real lon, int prec, std::string& georef) {
    if (abs(lat) > 90)
      throw GeographicErr("Latitude " + Utility::str(lat)
                          + "d not in [-90d, 90d]");
    if (Math::isnan(lat) || Math::isnan(lon)) {
      georef = "INVALID";
      return;
    }
    lon = Math::AngNormalize(lon); // lon in [-180,180)
    if (lat == 90) lat *= (1 - numeric_limits<real>::epsilon() / 2);
    prec = max(-1, min(int(maxprec_), prec));
    if (prec == 1) ++prec;      // Disallow prec = 1
    // The C++ standard mandates 64 bits for long long.  But
    // check, to make sure.
    GEOGRAPHICLIB_STATIC_ASSERT(numeric_limits<long long>::digits >= 45,
                                "long long not wide enough to store 21600e9");
    const long long m = 60000000000LL;
    long long
      x = (long long)(floor(lon * real(m))) - lonorig_ * m,
      y = (long long)(floor(lat * real(m))) - latorig_ * m;
    int ilon = int(x / m); int ilat = int(y / m);
    char georef1[maxlen_];
    georef1[0] = lontile_[ilon / tile_];
    georef1[1] = lattile_[ilat / tile_];
    if (prec >= 0) {
      georef1[2] = degrees_[ilon % tile_];
      georef1[3] = degrees_[ilat % tile_];
      if (prec > 0) {
        x -= m * ilon; y -= m * ilat;
        long long d = (long long)pow(real(base_), maxprec_ - prec);
        x /= d; y /= d;
        for (int c = prec; c--;) {
          georef1[baselen_ + c       ] = digits_[x % base_]; x /= base_;
          georef1[baselen_ + c + prec] = digits_[y % base_]; y /= base_;
        }
      }
    }
    georef.resize(baselen_ + 2 * prec);
    copy(georef1, georef1 + baselen_ + 2 * prec, georef.begin());
  }

  void Georef::Reverse(const std::string& georef, real& lat, real& lon,
                        int& prec, bool centerp) {
    int len = int(georef.length());
    if (len >= 3 &&
        toupper(georef[0]) == 'I' &&
        toupper(georef[1]) == 'N' &&
        toupper(georef[2]) == 'V') {
      lat = lon = Math::NaN();
      return;
    }
    if (len < baselen_ - 2)
      throw GeographicErr("Georef must start with at least 2 letters "
                          + georef);
    int prec1 = (2 + len - baselen_) / 2 - 1;
    int k;
    k = Utility::lookup(lontile_, georef[0]);
    if (k < 0)
      throw GeographicErr("Bad longitude tile letter in georef " + georef);
    real lon1 = k + lonorig_ / tile_;
    k = Utility::lookup(lattile_, georef[1]);
    if (k < 0)
      throw GeographicErr("Bad latitude tile letter in georef " + georef);
    real lat1 = k + latorig_ / tile_;
    real unit = 1;
    if (len > 2) {
      unit *= tile_;
      k = Utility::lookup(degrees_, georef[2]);
      if (k < 0)
        throw GeographicErr("Bad longitude degree letter in georef " + georef);
      lon1 = lon1 * tile_ + k;
      if (len < 4)
        throw GeographicErr("Missing latitude degree letter in georef "
                            + georef);
      k = Utility::lookup(degrees_, georef[3]);
      if (k < 0)
        throw GeographicErr("Bad latitude degree letter in georef " + georef);
      lat1 = lat1 * tile_ + k;
      if (prec1 > 0) {
        if (georef.find_first_not_of(digits_, baselen_) != string::npos)
          throw GeographicErr("Non digits in trailing portion of georef "
                              + georef.substr(baselen_));
        if (len % 2)
          throw GeographicErr("Georef must end with an even number of digits "
                              + georef.substr(baselen_));
        if (prec1 == 1)
          throw GeographicErr("Georef needs at least 4 digits for minutes "
                              + georef.substr(baselen_));
        if (prec1 > maxprec_)
          throw GeographicErr("More than " + Utility::str(2*maxprec_)
                              + " digits in georef "
                              + georef.substr(baselen_));
        for (int i = 0; i < prec1; ++i) {
          int m = i ? base_ : 6;
          unit *= m;
          int
            x = Utility::lookup(digits_, georef[baselen_ + i]),
            y = Utility::lookup(digits_, georef[baselen_ + i + prec1]);
          if (!(i || (x < m && y < m)))
            throw GeographicErr("Minutes terms in georef must be less than 60 "
                                + georef.substr(baselen_));
          lon1 = m * lon1 + x;
          lat1 = m * lat1 + y;
        }
      }
    }
    if (centerp) {
      unit *= 2; lat1 = 2 * lat1 + 1; lon1 = 2 * lon1 + 1;
    }
    lat = (tile_ * lat1) / unit;
    lon = (tile_ * lon1) / unit;
    prec = prec1;
  }

} // namespace GeographicLib
