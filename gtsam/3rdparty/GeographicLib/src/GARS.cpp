/**
 * \file GARS.cpp
 * \brief Implementation for GeographicLib::GARS class
 *
 * Copyright (c) Charles Karney (2015-2017) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <GeographicLib/GARS.hpp>
#include <GeographicLib/Utility.hpp>

namespace GeographicLib {

  using namespace std;

  const char* const GARS::digits_ = "0123456789";
  const char* const GARS::letters_ = "ABCDEFGHJKLMNPQRSTUVWXYZ";

  void GARS::Forward(real lat, real lon, int prec, std::string& gars) {
    if (abs(lat) > 90)
      throw GeographicErr("Latitude " + Utility::str(lat)
                          + "d not in [-90d, 90d]");
    if (Math::isnan(lat) || Math::isnan(lon)) {
      gars = "INVALID";
      return;
    }
    lon = Math::AngNormalize(lon); // lon in [-180,180)
    if (lat == 90) lat *= (1 - numeric_limits<real>::epsilon() / 2);
    prec = max(0, min(int(maxprec_), prec));
    int
      x = int(floor(lon * m_)) - lonorig_ * m_,
      y = int(floor(lat * m_)) - latorig_ * m_,
      ilon = x * mult1_ / m_,
      ilat = y * mult1_ / m_;
    x -= ilon * m_ / mult1_; y -= ilat * m_ / mult1_;
    char gars1[maxlen_];
    ++ilon;
    for (int c = lonlen_; c--;) {
      gars1[c] = digits_[ ilon % baselon_]; ilon /= baselon_;
    }
    for (int c = latlen_; c--;) {
      gars1[lonlen_ + c] = letters_[ilat % baselat_]; ilat /= baselat_;
    }
    if (prec > 0) {
      ilon = x / mult3_; ilat = y / mult3_;
      gars1[baselen_] = digits_[mult2_ * (mult2_ - 1 - ilat) + ilon + 1];
      if (prec > 1) {
        ilon = x % mult3_; ilat = y % mult3_;
        gars1[baselen_ + 1] = digits_[mult3_ * (mult3_ - 1 - ilat) + ilon + 1];
      }
    }
    gars.resize(baselen_ + prec);
    copy(gars1, gars1 + baselen_ + prec, gars.begin());
  }

  void GARS::Reverse(const std::string& gars, real& lat, real& lon,
                        int& prec, bool centerp) {
    int len = int(gars.length());
    if (len >= 3 &&
        toupper(gars[0]) == 'I' &&
        toupper(gars[1]) == 'N' &&
        toupper(gars[2]) == 'V') {
      lat = lon = Math::NaN();
      return;
    }
    if (len < baselen_)
      throw GeographicErr("GARS must have at least 5 characters " + gars);
    if (len > maxlen_)
      throw GeographicErr("GARS can have at most 7 characters " + gars);
    int prec1 = len - baselen_;
    int ilon = 0;
    for (int c = 0; c < lonlen_; ++c) {
      int k = Utility::lookup(digits_, gars[c]);
      if (k < 0)
        throw GeographicErr("GARS must start with 3 digits " + gars);
      ilon = ilon * baselon_ + k;
    }
    if (!(ilon >= 1 && ilon <= 720))
        throw GeographicErr("Initial digits in GARS must lie in [1, 720] " +
                            gars);
    --ilon;
    int ilat = 0;
    for (int c = 0; c < latlen_; ++c) {
      int k = Utility::lookup(letters_, gars[lonlen_ + c]);
      if (k < 0)
        throw GeographicErr("Illegal letters in GARS " + gars.substr(3,2));
      ilat = ilat * baselat_ + k;
    }
    if (!(ilat < 360))
      throw  GeographicErr("GARS letters must lie in [AA, QZ] " + gars);
    real
      unit = mult1_,
      lat1 = ilat + latorig_ * unit,
      lon1 = ilon + lonorig_ * unit;
    if (prec1 > 0) {
      int k = Utility::lookup(digits_, gars[baselen_]);
      if (!(k >= 1 && k <= mult2_ * mult2_))
        throw GeographicErr("6th character in GARS must [1, 4] " + gars);
      --k;
      unit *= mult2_;
      lat1 = mult2_ * lat1 + (mult2_ - 1 - k / mult2_);
      lon1 = mult2_ * lon1 + (k % mult2_);
      if (prec1 > 1) {
        k = Utility::lookup(digits_, gars[baselen_ + 1]);
        if (!(k >= 1 /* && k <= mult3_ * mult3_ */))
          throw GeographicErr("7th character in GARS must [1, 9] " + gars);
        --k;
        unit *= mult3_;
        lat1 = mult3_ * lat1 + (mult3_ - 1 - k / mult3_);
        lon1 = mult3_ * lon1 + (k % mult3_);
      }
    }
    if (centerp) {
      unit *= 2; lat1 = 2 * lat1 + 1; lon1 = 2 * lon1 + 1;
    }
    lat = lat1 / unit;
    lon = lon1 / unit;
    prec = prec1;
  }

} // namespace GeographicLib
