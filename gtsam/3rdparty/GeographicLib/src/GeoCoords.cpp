/**
 * \file GeoCoords.cpp
 * \brief Implementation for GeographicLib::GeoCoords class
 *
 * Copyright (c) Charles Karney (2008-2017) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <GeographicLib/GeoCoords.hpp>
#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/DMS.hpp>
#include <GeographicLib/Utility.hpp>

namespace GeographicLib {

  using namespace std;

  void GeoCoords::Reset(const std::string& s, bool centerp, bool longfirst) {
    vector<string> sa;
    const char* spaces = " \t\n\v\f\r,"; // Include comma as a space
    for (string::size_type pos0 = 0, pos1; pos0 != string::npos;) {
      pos1 = s.find_first_not_of(spaces, pos0);
      if (pos1 == string::npos)
        break;
      pos0 = s.find_first_of(spaces, pos1);
      sa.push_back(s.substr(pos1, pos0 == string::npos ? pos0 : pos0 - pos1));
    }
    if (sa.size() == 1) {
      int prec;
      MGRS::Reverse(sa[0], _zone, _northp, _easting, _northing, prec, centerp);
      UTMUPS::Reverse(_zone, _northp, _easting, _northing,
                      _lat, _long, _gamma, _k);
    } else if (sa.size() == 2) {
      DMS::DecodeLatLon(sa[0], sa[1], _lat, _long, longfirst);
      _long = Math::AngNormalize(_long);
      UTMUPS::Forward( _lat, _long,
                       _zone, _northp, _easting, _northing, _gamma, _k);
    } else if (sa.size() == 3) {
      unsigned zoneind, coordind;
      if (sa[0].size() > 0 && isalpha(sa[0][sa[0].size() - 1])) {
        zoneind = 0;
        coordind = 1;
      } else if (sa[2].size() > 0 && isalpha(sa[2][sa[2].size() - 1])) {
        zoneind = 2;
        coordind = 0;
      } else
        throw GeographicErr("Neither " + sa[0] + " nor " + sa[2]
                            + " of the form UTM/UPS Zone + Hemisphere"
                            + " (ex: 38n, 09s, n)");
      UTMUPS::DecodeZone(sa[zoneind], _zone, _northp);
      for (unsigned i = 0; i < 2; ++i)
        (i ? _northing : _easting) = Utility::val<real>(sa[coordind + i]);
      UTMUPS::Reverse(_zone, _northp, _easting, _northing,
                      _lat, _long, _gamma, _k);
      FixHemisphere();
    } else
      throw GeographicErr("Coordinate requires 1, 2, or 3 elements");
    CopyToAlt();
  }

  string GeoCoords::GeoRepresentation(int prec, bool longfirst) const {
    prec = max(0, min(9 + Math::extra_digits(), prec) + 5);
    ostringstream os;
    os << fixed << setprecision(prec);
    real a = longfirst ? _long : _lat;
    real b = longfirst ? _lat : _long;
    if (!Math::isnan(a))
      os << a;
    else
      os << "nan";
    os << " ";
    if (!Math::isnan(b))
      os << b;
    else
      os << "nan";
    return os.str();
  }

  string GeoCoords::DMSRepresentation(int prec, bool longfirst,
                                      char dmssep) const {
    prec = max(0, min(10 + Math::extra_digits(), prec) + 5);
    return DMS::Encode(longfirst ? _long : _lat, unsigned(prec),
                       longfirst ? DMS::LONGITUDE : DMS::LATITUDE, dmssep) +
      " " + DMS::Encode(longfirst ? _lat : _long, unsigned(prec),
                        longfirst ? DMS::LATITUDE : DMS::LONGITUDE, dmssep);
  }

  string GeoCoords::MGRSRepresentation(int prec) const {
    // Max precision is um
    prec = max(-1, min(6, prec) + 5);
    string mgrs;
    MGRS::Forward(_zone, _northp, _easting, _northing, _lat, prec, mgrs);
    return mgrs;
  }

  string GeoCoords::AltMGRSRepresentation(int prec) const {
    // Max precision is um
    prec = max(-1, min(6, prec) + 5);
    string mgrs;
    MGRS::Forward(_alt_zone, _northp, _alt_easting, _alt_northing, _lat, prec,
                  mgrs);
    return mgrs;
  }

  void GeoCoords::UTMUPSString(int zone, bool northp,
                               real easting, real northing, int prec,
                               bool abbrev, std::string& utm) {
    ostringstream os;
    prec = max(-5, min(9 + Math::extra_digits(), prec));
    // Need extra real because, since C++11, pow(float, int) returns double
    real scale = prec < 0 ? real(pow(real(10), -prec)) : real(1);
    os << UTMUPS::EncodeZone(zone, northp, abbrev) << fixed << setfill('0');
    if (Math::isfinite(easting)) {
      os << " " << Utility::str(easting / scale, max(0, prec));
      if (prec < 0 && abs(easting / scale) > real(0.5))
        os << setw(-prec) << 0;
    } else
      os << " nan";
    if (Math::isfinite(northing)) {
      os << " " << Utility::str(northing / scale, max(0, prec));
      if (prec < 0 && abs(northing / scale) > real(0.5))
        os << setw(-prec) << 0;
    } else
      os << " nan";
    utm = os.str();
  }

  string GeoCoords::UTMUPSRepresentation(int prec, bool abbrev) const {
    string utm;
    UTMUPSString(_zone, _northp, _easting, _northing, prec, abbrev, utm);
    return utm;
  }

  string GeoCoords::UTMUPSRepresentation(bool northp, int prec,
                                         bool abbrev) const {
    real e, n;
    int z;
    UTMUPS::Transfer(_zone, _northp, _easting, _northing,
                     _zone,  northp,  e,        n,       z);
    string utm;
    UTMUPSString(_zone, northp, e, n, prec, abbrev, utm);
    return utm;
  }

  string GeoCoords::AltUTMUPSRepresentation(int prec, bool abbrev) const {
    string utm;
    UTMUPSString(_alt_zone, _northp, _alt_easting, _alt_northing, prec,
                 abbrev, utm);
    return utm;
  }

  string GeoCoords::AltUTMUPSRepresentation(bool northp, int prec,
                                            bool abbrev) const {
    real e, n;
    int z;
    UTMUPS::Transfer(_alt_zone, _northp, _alt_easting, _alt_northing,
                     _alt_zone,  northp,      e,            n,       z);
    string utm;
    UTMUPSString(_alt_zone, northp, e, n, prec, abbrev, utm);
    return utm;
  }

  void GeoCoords::FixHemisphere() {
    if (_lat == 0 || (_northp && _lat >= 0) || (!_northp && _lat < 0) ||
        Math::isnan(_lat))
      // Allow either hemisphere for equator
      return;
    if (_zone != UTMUPS::UPS) {
      _northing += (_northp ? 1 : -1) * UTMUPS::UTMShift();
      _northp = !_northp;
    } else
      throw GeographicErr("Hemisphere mixup");
  }

} // namespace GeographicLib
