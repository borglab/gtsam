/**
 * \file UTMUPS.cpp
 * \brief Implementation for GeographicLib::UTMUPS class
 *
 * Copyright (c) Charles Karney (2008-2012) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * http://geographiclib.sourceforge.net/
 **********************************************************************/

#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/PolarStereographic.hpp>
#include <GeographicLib/TransverseMercator.hpp>
#include <GeographicLib/Utility.hpp>

namespace GeographicLib {

  using namespace std;

  const Math::real UTMUPS::falseeasting_[4] =
    { MGRS::upseasting_ * MGRS::tile_, MGRS::upseasting_ * MGRS::tile_,
      MGRS::utmeasting_ * MGRS::tile_, MGRS::utmeasting_ * MGRS::tile_ };
  const Math::real UTMUPS::falsenorthing_[4] =
    { MGRS::upseasting_ * MGRS::tile_, MGRS::upseasting_ * MGRS::tile_,
      MGRS::maxutmSrow_ * MGRS::tile_, MGRS::minutmNrow_ * MGRS::tile_ };
  const Math::real UTMUPS::mineasting_[4] =
    { MGRS::minupsSind_ * MGRS::tile_, MGRS::minupsNind_ * MGRS::tile_,
      MGRS::minutmcol_ * MGRS::tile_, MGRS::minutmcol_ * MGRS::tile_ };
  const Math::real UTMUPS::maxeasting_[4] =
    { MGRS::maxupsSind_ * MGRS::tile_, MGRS::maxupsNind_ * MGRS::tile_,
      MGRS::maxutmcol_ * MGRS::tile_, MGRS::maxutmcol_ * MGRS::tile_ };
  const Math::real UTMUPS::minnorthing_[4] =
    { MGRS::minupsSind_ * MGRS::tile_, MGRS::minupsNind_ * MGRS::tile_,
      MGRS::minutmSrow_ * MGRS::tile_,
      (MGRS::minutmNrow_ + MGRS::minutmSrow_ - MGRS::maxutmSrow_)
      * MGRS::tile_ };
  const Math::real UTMUPS::maxnorthing_[4] =
    { MGRS::maxupsSind_ * MGRS::tile_, MGRS::maxupsNind_ * MGRS::tile_,
      (MGRS::maxutmSrow_ + MGRS::maxutmNrow_ - MGRS::minutmNrow_) * MGRS::tile_,
      MGRS::maxutmNrow_ * MGRS::tile_ };

  int UTMUPS::StandardZone(real lat, real lon, int setzone) {
    if (!(setzone >= MINPSEUDOZONE && setzone <= MAXZONE))
      throw GeographicErr("Illegal zone requested " + Utility::str(setzone));
    if (setzone >= MINZONE || setzone == INVALID)
      return setzone;
    if (Math::isnan(lat) || Math::isnan(lon)) // Check if lat or lon is a NaN
      return INVALID;
    if (setzone == UTM || (lat >= -80 && lat < 84)) {
      // Assume lon is in [-540, 540).
      int ilon = int(floor(lon));
      if (ilon >= 180)
        ilon -= 360;
      else if (ilon < -180)
        ilon += 360;
      int zone = (ilon + 186)/6;
      int band = MGRS::LatitudeBand(lat);
      if (band == 7 && zone == 31 && ilon >= 3)
        zone = 32;
      else if (band == 9 && ilon >= 0 && ilon < 42)
        zone = 2 * ((ilon + 183)/12) + 1;
      return zone;
    } else
      return UPS;
  }

  void UTMUPS::Forward(real lat, real lon,
                       int& zone, bool& northp, real& x, real& y,
                       real& gamma, real& k,
                       int setzone, bool mgrslimits) {
    CheckLatLon(lat, lon);
    bool northp1 = lat >= 0;
    int zone1 = StandardZone(lat, lon, setzone);
    if (zone1 == INVALID) {
      zone = zone1;
      northp = northp1;
      x = y = gamma = k = Math::NaN<real>();
      return;
    }
    real x1, y1, gamma1, k1;
    bool utmp = zone1 != UPS;
    if (utmp) {
      real
        lon0 = CentralMeridian(zone1),
        dlon = lon - lon0;
      dlon = abs(dlon - 360 * floor((dlon + 180)/360));
      if (dlon > 60)
        // Check isn't really necessary because CheckCoords catches this case.
        // But this allows a more meaningful error message to be given.
        throw GeographicErr("Longitude " + Utility::str(lon)
                            + "d more than 60d from center of UTM zone "
                            + Utility::str(zone1));
      TransverseMercator::UTM.Forward(lon0, lat, lon, x1, y1, gamma1, k1);
    } else {
      if (abs(lat) < 70)
        // Check isn't really necessary ... (see above).
        throw GeographicErr("Latitude " + Utility::str(lat)
                            + "d more than 20d from "
                            + (northp1 ? "N" : "S") + " pole");
      PolarStereographic::UPS.Forward(northp1, lat, lon, x1, y1, gamma1, k1);
    }
    int ind = (utmp ? 2 : 0) + (northp1 ? 1 : 0);
    x1 += falseeasting_[ind];
    y1 += falsenorthing_[ind];
    if (! CheckCoords(zone1 != UPS, northp1, x1, y1, mgrslimits, false) )
      throw GeographicErr("Latitude " + Utility::str(lat)
                          + ", longitude " + Utility::str(lon)
                          + " out of legal range for "
                          + (utmp ? "UTM zone " + Utility::str(zone1) : "UPS"));
    zone = zone1;
    northp = northp1;
    x = x1;
    y = y1;
    gamma = gamma1;
    k = k1;
  }

  void UTMUPS::Reverse(int zone, bool northp, real x, real y,
                       real& lat, real& lon, real& gamma, real& k,
                       bool mgrslimits) {
    if (zone == INVALID || Math::isnan(x) || Math::isnan(y)) {
      lat = lon = gamma = k = Math::NaN<real>();
      return;
    }
    if (!(zone >= MINZONE && zone <= MAXZONE))
      throw GeographicErr("Zone " + Utility::str(zone)
                          + " not in range [0, 60]");
    bool utmp = zone != UPS;
    CheckCoords(utmp, northp, x, y, mgrslimits);
    int ind = (utmp ? 2 : 0) + (northp ? 1 : 0);
    x -= falseeasting_[ind];
    y -= falsenorthing_[ind];
    if (utmp)
      TransverseMercator::UTM.Reverse(CentralMeridian(zone),
                                      x, y, lat, lon, gamma, k);
    else
      PolarStereographic::UPS.Reverse(northp, x, y, lat, lon, gamma, k);
  }

  void UTMUPS::CheckLatLon(real lat, real lon) {
    if (abs(lat) > 90)
      throw GeographicErr("Latitude " + Utility::str(lat)
                          + "d not in [-90d, 90d]");
    if (lon < -540 || lon >= 540)
      throw GeographicErr("Longitude " + Utility::str(lon)
                          + "d not in [-540d, 540d)");
    }

  bool UTMUPS::CheckCoords(bool utmp, bool northp, real x, real y,
                           bool mgrslimits, bool throwp) {
    // Limits are all multiples of 100km and are all closed on the both ends.
    // Failure tests are such that NaNs succeed.
    real slop = mgrslimits ? 0 : MGRS::tile_;
    int ind = (utmp ? 2 : 0) + (northp ? 1 : 0);
    if (x < mineasting_[ind] - slop || x > maxeasting_[ind] + slop) {
      if (!throwp) return false;
      throw GeographicErr("Easting " + Utility::str(x/1000) + "km not in "
                          + (mgrslimits ? "MGRS/" : "")
                          + (utmp ? "UTM" : "UPS") + " range for "
                          + (northp ? "N" : "S" ) + " hemisphere ["
                          + Utility::str((mineasting_[ind] - slop)/1000)
                          + "km, "
                          + Utility::str((maxeasting_[ind] + slop)/1000)
                          + "km]");
    }
    if (y < minnorthing_[ind] - slop || y > maxnorthing_[ind] + slop) {
      if (!throwp) return false;
      throw GeographicErr("Northing " + Utility::str(y/1000) + "km not in "
                          + (mgrslimits ? "MGRS/" : "")
                          + (utmp ? "UTM" : "UPS") + " range for "
                          + (northp ? "N" : "S" ) + " hemisphere ["
                          + Utility::str((minnorthing_[ind] - slop)/1000)
                          + "km, "
                          + Utility::str((maxnorthing_[ind] + slop)/1000)
                          + "km]");
    }
    return true;
  }

  void UTMUPS::Transfer(int zonein, bool northpin, real xin, real yin,
                        int zoneout, bool northpout, real& xout, real& yout,
                        int& zone) {
    bool northp = northpin;
    if (zonein != zoneout) {
      // Determine lat, lon
      real lat, lon;
      GeographicLib::UTMUPS::Reverse(zonein, northpin, xin, yin, lat, lon);
      // Try converting to zoneout
      real x, y;
      int zone1;
      GeographicLib::UTMUPS::Forward(lat, lon, zone1, northp, x, y,
                                     zoneout == UTMUPS::MATCH
                                     ? zonein : zoneout);
      if (zone1 == 0 && northp != northpout)
        throw GeographicErr
          ("Attempt to transfer UPS coordinates between hemispheres");
      zone = zone1;
      xout = x;
      yout = y;
    } else {
      if (zoneout == 0 && northp != northpout)
        throw GeographicErr
          ("Attempt to transfer UPS coordinates between hemispheres");
      zone = zoneout;
      xout = xin;
      yout = yin;
    }
    if (northp != northpout)
      // Can't get here if UPS
      yout += (northpout ? -1 : 1) * MGRS::utmNshift_;
    return;
  }

  void UTMUPS::DecodeZone(const std::string& zonestr, int& zone, bool& northp) {
    unsigned zlen = unsigned(zonestr.size());
    if (zlen == 0)
      throw GeographicErr("Empty zone specification");
    if (zlen > 3)
      throw GeographicErr("More than 3 characters in zone specification "
                          + zonestr);
    if (zlen == 3 &&
        toupper(zonestr[0]) == 'I' &&
        toupper(zonestr[1]) == 'N' &&
        toupper(zonestr[2]) == 'V') {
      zone = INVALID;
      northp = false;
      return;
    }
    char hemi = char(toupper(zonestr[zlen - 1]));
    bool northp1 = hemi == 'N';
    if (! (northp1 || hemi == 'S'))
      throw GeographicErr(string("Illegal hemisphere letter ") + hemi + " in "
                          + zonestr + ", specify N or S");
    if (zlen == 1)
      zone = UPS;
    else {
      const char* c = zonestr.c_str();
      char* q;
      int zone1 = strtol(c, &q, 10);
      if (q == c)
        throw GeographicErr("No zone number found in " + zonestr);
      if (q - c != int(zlen) - 1)
        throw GeographicErr("Extra text " +
                            zonestr.substr(q - c, int(zlen) - 1 - (q - c)) +
                            " in UTM/UPS zone " + zonestr);
      if (zone1 == UPS)
        // Don't allow 0N as an alternative to N for UPS coordinates
        throw GeographicErr("Illegal zone 0 in " + zonestr +
                            ", use just " + hemi + " for UPS");
      if (!(zone1 >= MINUTMZONE && zone1 <= MAXUTMZONE))
        throw GeographicErr("Zone " + Utility::str(zone1)
                            + " not in range [1, 60]");
      zone = zone1;
    }
    northp = northp1;
  }

  std::string UTMUPS::EncodeZone(int zone, bool northp) {
    if (zone == INVALID)
      return string("INV");
    if (!(zone >= MINZONE && zone <= MAXZONE))
        throw GeographicErr("Zone " + Utility::str(zone)
                            + " not in range [0, 60]");
    ostringstream os;
    if (zone != UPS)
      os << setfill('0') << setw(2) << zone;
    os << (northp ? 'N' : 'S');
    return os.str();
  }

  void UTMUPS::DecodeEPSG(int epsg, int& zone, bool& northp) throw() {
    if (epsg >= epsg01N && epsg <= epsg60N) {
      zone = epsg - epsg01N + 1;
      northp = true;
    } else if (epsg == epsgN) {
      zone = UPS;
      northp = true;
    } else if (epsg >= epsg01S && epsg <= epsg60S) {
      zone = epsg - epsg01S + 1;
      northp = false;
    } else if (epsg == epsgS) {
      zone = UPS;
      northp = false;
    } else {
      zone = INVALID;
      northp = false;
    }
  }

  int UTMUPS::EncodeEPSG(int zone, bool northp) throw() {
    int epsg = -1;
    if (zone == UPS)
      epsg = epsgS;
    else if (zone >= MINUTMZONE && zone <= MAXUTMZONE)
      epsg = epsg + (zone - MINUTMZONE) + epsg01S;
    if (epsg >= 0 && northp)
      epsg += epsgN - epsgS;
    return epsg;
  }

  Math::real UTMUPS::UTMShift() throw() { return real(MGRS::utmNshift_); }

} // namespace GeographicLib
