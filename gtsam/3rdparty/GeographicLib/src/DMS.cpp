/**
 * \file DMS.cpp
 * \brief Implementation for GeographicLib::DMS class
 *
 * Copyright (c) Charles Karney (2008-2011) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * http://geographiclib.sourceforge.net/
 **********************************************************************/

#include <GeographicLib/DMS.hpp>
#include <GeographicLib/Utility.hpp>

#if defined(_MSC_VER)
// Squelch warnings about constant conditional expressions
#  pragma warning (disable: 4127)
#endif

namespace GeographicLib {

  using namespace std;

  const string DMS::hemispheres_ = "SNWE";
  const string DMS::signs_ = "-+";
  const string DMS::digits_ = "0123456789";
  const string DMS::dmsindicators_ = "D'\":";
  const string DMS::components_[] = {"degrees", "minutes", "seconds"};

  Math::real DMS::Decode(const std::string& dms, flag& ind) {
    string errormsg;
    string dmsa = dms;
    replace(dmsa, "\xc2\xb0", 'd');      // U+00b0 degree symbol
    replace(dmsa, "\xc2\xba", 'd');      // U+00ba alt symbol
    replace(dmsa, "\xe2\x81\xb0", 'd');  // U+2070 sup zero
    replace(dmsa, "\xcb\x9a", 'd');      // U+02da ring above
    replace(dmsa, "\xe2\x80\xb2", '\''); // U+2032 prime
    replace(dmsa, "\xc2\xb4", '\'');     // U+00b4 acute accent
    replace(dmsa, "\xe2\x80\x99", '\''); // U+2019 right single quote
    replace(dmsa, "\xe2\x80\xb3", '"');  // U+2033 double prime
    replace(dmsa, "\xe2\x80\x9d", '"');  // U+201d right double quote
    replace(dmsa, "\xb0", 'd');          // 0xb0 bare degree symbol
    replace(dmsa, "\xba", 'd');          // 0xba bare alt symbol
    replace(dmsa, "\xb4", '\'');         // 0xb4 bare acute accent
    replace(dmsa, "''", '"');            // '' -> "
    do {                       // Executed once (provides the ability to break)
      int sign = 1;
      unsigned
        beg = 0,
        end = unsigned(dmsa.size());
      while (beg < end && isspace(dmsa[beg]))
        ++beg;
      while (beg < end && isspace(dmsa[end - 1]))
        --end;
      flag ind1 = NONE;
      int k = -1;
      if (end > beg && (k = Utility::lookup(hemispheres_, dmsa[beg])) >= 0) {
        ind1 = (k / 2) ? LONGITUDE : LATITUDE;
        sign = k % 2 ? 1 : -1;
        ++beg;
      }
      if (end > beg && (k = Utility::lookup(hemispheres_, dmsa[end-1])) >= 0) {
        if (k >= 0) {
          if (ind1 != NONE) {
            if (toupper(dmsa[beg - 1]) == toupper(dmsa[end - 1]))
              errormsg = "Repeated hemisphere indicators "
                + Utility::str(dmsa[beg - 1])
                + " in " + dmsa.substr(beg - 1, end - beg + 1);
            else
              errormsg = "Contradictory hemisphere indicators "
                + Utility::str(dmsa[beg - 1]) + " and "
                + Utility::str(dmsa[end - 1]) + " in "
                + dmsa.substr(beg - 1, end - beg + 1);
            break;
          }
          ind1 = (k / 2) ? LONGITUDE : LATITUDE;
          sign = k % 2 ? 1 : -1;
          --end;
        }
      }
      if (end > beg && (k = Utility::lookup(signs_, dmsa[beg])) >= 0) {
        if (k >= 0) {
          sign *= k ? 1 : -1;
          ++beg;
        }
      }
      if (end == beg) {
        errormsg = "Empty or incomplete DMS string " + dmsa;
        break;
      }
      real ipieces[] = {0, 0, 0};
      real fpieces[] = {0, 0, 0};
      unsigned npiece = 0;
      real icurrent = 0;
      real fcurrent = 0;
      unsigned ncurrent = 0, p = beg;
      bool pointseen = false;
      unsigned digcount = 0, intcount = 0;
      while (p < end) {
        char x = dmsa[p++];
        if ((k = Utility::lookup(digits_, x)) >= 0) {
          ++ncurrent;
          if (digcount > 0)
            ++digcount;         // Count of decimal digits
          else {
            icurrent = 10 * icurrent + k;
            ++intcount;
          }
        } else if (x == '.') {
          if (pointseen) {
            errormsg = "Multiple decimal points in "
              + dmsa.substr(beg, end - beg);
            break;
          }
          pointseen = true;
          digcount = 1;
        } else if ((k = Utility::lookup(dmsindicators_, x)) >= 0) {
          if (k >= 3) {
            if (p == end) {
              errormsg = "Illegal for : to appear at the end of " +
                dmsa.substr(beg, end - beg);
              break;
            }
            k = npiece;
          }
          if (unsigned(k) == npiece - 1) {
            errormsg = "Repeated " + components_[k] +
              " component in " + dmsa.substr(beg, end - beg);
            break;
          } else if (unsigned(k) < npiece) {
            errormsg = components_[k] + " component follows "
              + components_[npiece - 1] + " component in "
              + dmsa.substr(beg, end - beg);
            break;
          }
          if (ncurrent == 0) {
            errormsg = "Missing numbers in " + components_[k] +
              " component of " + dmsa.substr(beg, end - beg);
            break;
          }
          if (digcount > 1) {
            istringstream s(dmsa.substr(p - intcount - digcount - 1,
                                        intcount + digcount));
            s >> fcurrent;
            icurrent = 0;
          }
          ipieces[k] = icurrent;
          fpieces[k] = icurrent + fcurrent;
          if (p < end) {
            npiece = k + 1;
            icurrent = fcurrent = 0;
            ncurrent = digcount = intcount = 0;
          }
        } else if (Utility::lookup(signs_, x) >= 0) {
          errormsg = "Internal sign in DMS string "
            + dmsa.substr(beg, end - beg);
          break;
        } else {
          errormsg = "Illegal character " + Utility::str(x) + " in DMS string "
            + dmsa.substr(beg, end - beg);
          break;
        }
      }
      if (!errormsg.empty())
        break;
      if (Utility::lookup(dmsindicators_, dmsa[p - 1]) < 0) {
        if (npiece >= 3) {
          errormsg = "Extra text following seconds in DMS string "
            + dmsa.substr(beg, end - beg);
          break;
        }
        if (ncurrent == 0) {
          errormsg = "Missing numbers in trailing component of "
            + dmsa.substr(beg, end - beg);
          break;
        }
        if (digcount > 1) {
          istringstream s(dmsa.substr(p - intcount - digcount,
                                      intcount + digcount));
          s >> fcurrent;
          icurrent = 0;
        }
        ipieces[npiece] = icurrent;
        fpieces[npiece] = icurrent + fcurrent;
      }
      if (pointseen && digcount == 0) {
        errormsg = "Decimal point in non-terminal component of "
          + dmsa.substr(beg, end - beg);
        break;
      }
      // Note that we accept 59.999999... even though it rounds to 60.
      if (ipieces[1] >= 60) {
        errormsg = "Minutes " + Utility::str(fpieces[1])
          + " not in range [0, 60)";
        break;
      }
      if (ipieces[2] >= 60) {
        errormsg = "Seconds " + Utility::str(fpieces[2])
          + " not in range [0, 60)";
        break;
      }
      ind = ind1;
      // Assume check on range of result is made by calling routine (which
      // might be able to offer a better diagnostic).
      return real(sign) * (fpieces[0] + (fpieces[1] + fpieces[2] / 60) / 60);
    } while (false);
    real val = Utility::nummatch<real>(dmsa);
    if (val == 0)
      throw GeographicErr(errormsg);
    else
      ind = NONE;
    return val;
  }

  void DMS::DecodeLatLon(const std::string& stra, const std::string& strb,
                         real& lat, real& lon, bool swaplatlong) {
    real a, b;
    flag ia, ib;
    a = Decode(stra, ia);
    b = Decode(strb, ib);
    if (ia == NONE && ib == NONE) {
      // Default to lat, long unless swaplatlong
      ia = swaplatlong ? LONGITUDE : LATITUDE;
      ib = swaplatlong ? LATITUDE : LONGITUDE;
    } else if (ia == NONE)
      ia = flag(LATITUDE + LONGITUDE - ib);
    else if (ib == NONE)
      ib = flag(LATITUDE + LONGITUDE - ia);
    if (ia == ib)
      throw GeographicErr("Both " + stra + " and "
                          + strb + " interpreted as "
                          + (ia == LATITUDE ? "latitudes" : "longitudes"));
    real
      lat1 = ia == LATITUDE ? a : b,
      lon1 = ia == LATITUDE ? b : a;
    if (abs(lat1) > 90)
      throw GeographicErr("Latitude " + Utility::str(lat1)
                          + "d not in [-90d, 90d]");
    if (lon1 < -540 || lon1 >= 540)
      throw GeographicErr("Longitude " + Utility::str(lon1)
                          + "d not in [-540d, 540d)");
    lon1 = Math::AngNormalize(lon1);
    lat = lat1;
    lon = lon1;
  }

  Math::real DMS::DecodeAngle(const std::string& angstr) {
    flag ind;
    real ang = Decode(angstr, ind);
    if (ind != NONE)
      throw GeographicErr("Arc angle " + angstr
                          + " includes a hemisphere, N/E/W/S");
    return ang;
  }

  Math::real DMS::DecodeAzimuth(const std::string& azistr) {
    flag ind;
    real azi = Decode(azistr, ind);
    if (ind == LATITUDE)
      throw GeographicErr("Azimuth " + azistr
                          + " has a latitude hemisphere, N/S");
    if (azi < -540 || azi >= 540)
      throw GeographicErr("Azimuth " + azistr + " not in range [-540d, 540d)");
    return Math::AngNormalize(azi);
  }

  string DMS::Encode(real angle, component trailing, unsigned prec, flag ind,
                     char dmssep) {
    // Assume check on range of input angle has been made by calling
    // routine (which might be able to offer a better diagnostic).
    if (!Math::isfinite(angle))
      return angle < 0 ? string("-inf") :
        (angle > 0 ? string("inf") : string("nan"));

    // 15 - 2 * trailing = ceiling(log10(2^53/90/60^trailing)).
    // This suffices to give full real precision for numbers in [-90,90]
    prec = min(15 + Math::extradigits - 2 * unsigned(trailing), prec);
    real scale = 1;
    for (unsigned i = 0; i < unsigned(trailing); ++i)
      scale *= 60;
    for (unsigned i = 0; i < prec; ++i)
      scale *= 10;
    if (ind == AZIMUTH)
      angle -= floor(angle/360) * 360;
    int sign = angle < 0 ? -1 : 1;
    angle *= sign;

    // Break off integer part to preserve precision in manipulation of
    // fractional part.
    real
      idegree = floor(angle),
      fdegree = floor((angle - idegree) * scale + real(0.5)) / scale;
    if (fdegree >= 1) {
      idegree += 1;
      fdegree -= 1;
    }
    real pieces[3] = {fdegree, 0, 0};
    for (unsigned i = 1; i <= unsigned(trailing); ++i) {
      real
        ip = floor(pieces[i - 1]),
        fp = pieces[i - 1] - ip;
      pieces[i] = fp * 60;
      pieces[i - 1] = ip;
    }
    pieces[0] += idegree;
    ostringstream s;
    s << fixed << setfill('0');
    if (ind == NONE && sign < 0)
      s << '-';
    switch (trailing) {
    case DEGREE:
      if (ind != NONE)
        s << setw(1 + min(int(ind), 2) + prec + (prec ? 1 : 0));
      s << setprecision(prec) << pieces[0];
      // Don't include degree designator (d) if it is the trailing component.
      break;
    default:
      if (ind != NONE)
        s << setw(1 + min(int(ind), 2));
      s << setprecision(0) << pieces[0]
        << (dmssep ? dmssep : char(tolower(dmsindicators_[0])));
      switch (trailing) {
      case MINUTE:
        s << setw(2 + prec + (prec ? 1 : 0)) << setprecision(prec) << pieces[1];
        if (!dmssep)
          s << char(tolower(dmsindicators_[1]));
        break;
      case SECOND:
        s << setw(2)
          << pieces[1] << (dmssep ? dmssep : char(tolower(dmsindicators_[1])))
          << setw(2 + prec + (prec ? 1 : 0)) << setprecision(prec) << pieces[2];
        if (!dmssep)
          s << char(tolower(dmsindicators_[2]));
        break;
      default:
        break;
      }
    }
    if (ind != NONE && ind != AZIMUTH)
      s << hemispheres_[(ind == LATITUDE ? 0 : 2) + (sign < 0 ? 0 : 1)];
    return s.str();
  }

} // namespace GeographicLib
