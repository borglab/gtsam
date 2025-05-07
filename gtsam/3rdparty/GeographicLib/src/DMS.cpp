/**
 * \file DMS.cpp
 * \brief Implementation for GeographicLib::DMS class
 *
 * Copyright (c) Charles Karney (2008-2022) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <GeographicLib/DMS.hpp>
#include <GeographicLib/Utility.hpp>

namespace GeographicLib {

  using namespace std;

  const char* const DMS::hemispheres_ = "SNWE";
  const char* const DMS::signs_ = "-+";
  const char* const DMS::digits_ = "0123456789";
  const char* const DMS::dmsindicators_ = "D'\":";
  const char* const DMS::components_[] = {"degrees", "minutes", "seconds"};

  // Replace all occurrences of pat by c.  If c is NULL remove pat.
  void DMS::replace(std::string& s, const std::string& pat, char c) {
    string::size_type p = 0;
    int count = c ? 1 : 0;
    while (true) {
      p = s.find(pat, p);
      if (p == string::npos)
        break;
      s.replace(p, pat.length(), count, c);
    }
  }

  Math::real DMS::Decode(const std::string& dms, flag& ind) {
    // Here's a table of the allowed characters

    // S unicode   dec  UTF-8      descripton

    // DEGREE
    // d U+0064    100  64         d
    // D U+0044     68  44         D
    // ° U+00b0    176  c2 b0      degree symbol
    // º U+00ba    186  c2 ba      alt symbol
    // ⁰ U+2070   8304  e2 81 b0   sup zero
    // ˚ U+02da    730  cb 9a      ring above
    // ∘ U+2218   8728  e2 88 98   compose function
    // * U+002a     42  2a         GRiD symbol for degrees

    // MINUTES
    // ' U+0027     39  27         apostrophe
    // ` U+0060     96  60         grave accent
    // ′ U+2032   8242  e2 80 b2   prime
    // ‵ U+2035   8245  e2 80 b5   back prime
    // ´ U+00b4    180  c2 b4      acute accent
    // ‘ U+2018   8216  e2 80 98   left single quote (also ext ASCII 0x91)
    // ’ U+2019   8217  e2 80 99   right single quote (also ext ASCII 0x92)
    // ‛ U+201b   8219  e2 80 9b   reversed-9 single quote
    // ʹ U+02b9    697  ca b9      modifier letter prime
    // ˊ U+02ca    714  cb 8a      modifier letter acute accent
    // ˋ U+02cb    715  cb 8b      modifier letter grave accent

    // SECONDS
    // " U+0022     34  22         quotation mark
    // ″ U+2033   8243  e2 80 b3   double prime
    // ‶ U+2036   8246  e2 80 b6   reversed double prime
    // ˝ U+02dd    733  cb 9d      double acute accent
    // “ U+201c   8220  e2 80 9c   left double quote (also ext ASCII 0x93)
    // ” U+201d   8221  e2 80 9d   right double quote (also ext ASCII 0x94)
    // ‟ U+201f   8223  e2 80 9f   reversed-9 double quote
    // ʺ U+02ba    698  ca ba      modifier letter double prime

    // PLUS
    // + U+002b     43  2b         plus sign
    // ➕ U+2795  10133  e2 9e 95   heavy plus
    //   U+2064   8292  e2 81 a4   invisible plus |⁤|

    // MINUS
    // - U+002d     45  2d         hyphen
    // ‐ U+2010   8208  e2 80 90   dash
    // ‑ U+2011   8209  e2 80 91   non-breaking hyphen
    // – U+2013   8211  e2 80 93   en dash (also ext ASCII 0x96)
    // — U+2014   8212  e2 80 94   em dash (also ext ASCII 0x97)
    // − U+2212   8722  e2 88 92   minus sign
    // ➖ U+2796  10134  e2 9e 96   heavy minus

    // IGNORED
    //   U+00a0    160  c2 a0      non-breaking space
    //   U+2007   8199  e2 80 87   figure space | |
    //   U+2009   8201  e2 80 89   thin space   | |
    //   U+200a   8202  e2 80 8a   hair space   | |
    //   U+200b   8203  e2 80 8b   invisible space |​|
    //   U+202f   8239  e2 80 af   narrow space | |
    //   U+2063   8291  e2 81 a3   invisible separator |⁣|
    // « U+00ab    171  c2 ab      left guillemot (for cgi-bin)
    // » U+00bb    187  c2 bb      right guillemot (for cgi-bin)

    string dmsa = dms;
    replace(dmsa, "\xc2\xb0",     'd' ); // U+00b0 degree symbol
    replace(dmsa, "\xc2\xba",     'd' ); // U+00ba alt symbol
    replace(dmsa, "\xe2\x81\xb0", 'd' ); // U+2070 sup zero
    replace(dmsa, "\xcb\x9a",     'd' ); // U+02da ring above
    replace(dmsa, "\xe2\x88\x98", 'd' ); // U+2218 compose function

    replace(dmsa, "\xe2\x80\xb2", '\''); // U+2032 prime
    replace(dmsa, "\xe2\x80\xb5", '\''); // U+2035 back prime
    replace(dmsa, "\xc2\xb4",     '\''); // U+00b4 acute accent
    replace(dmsa, "\xe2\x80\x98", '\''); // U+2018 left single quote
    replace(dmsa, "\xe2\x80\x99", '\''); // U+2019 right single quote
    replace(dmsa, "\xe2\x80\x9b", '\''); // U+201b reversed-9 single quote
    replace(dmsa, "\xca\xb9",     '\''); // U+02b9 modifier letter prime
    replace(dmsa, "\xcb\x8a",     '\''); // U+02ca modifier letter acute accent
    replace(dmsa, "\xcb\x8b",     '\''); // U+02cb modifier letter grave accent

    replace(dmsa, "\xe2\x80\xb3", '"' ); // U+2033 double prime
    replace(dmsa, "\xe2\x80\xb6", '"' ); // U+2036 reversed double prime
    replace(dmsa, "\xcb\x9d",     '"' ); // U+02dd double acute accent
    replace(dmsa, "\xe2\x80\x9c", '"' ); // U+201c left double quote
    replace(dmsa, "\xe2\x80\x9d", '"' ); // U+201d right double quote
    replace(dmsa, "\xe2\x80\x9f", '"' ); // U+201f reversed-9 double quote
    replace(dmsa, "\xca\xba",     '"' ); // U+02ba modifier letter double prime

    replace(dmsa, "\xe2\x9e\x95", '+' ); // U+2795 heavy plus
    replace(dmsa, "\xe2\x81\xa4", '+' ); // U+2064 invisible plus

    replace(dmsa, "\xe2\x80\x90", '-' ); // U+2010 dash
    replace(dmsa, "\xe2\x80\x91", '-' ); // U+2011 non-breaking hyphen
    replace(dmsa, "\xe2\x80\x93", '-' ); // U+2013 en dash
    replace(dmsa, "\xe2\x80\x94", '-' ); // U+2014 em dash
    replace(dmsa, "\xe2\x88\x92", '-' ); // U+2212 minus sign
    replace(dmsa, "\xe2\x9e\x96", '-' ); // U+2796 heavy minus

    replace(dmsa, "\xc2\xa0",     '\0'); // U+00a0 non-breaking space
    replace(dmsa, "\xe2\x80\x87", '\0'); // U+2007 figure space
    replace(dmsa, "\xe2\x80\x89", '\0'); // U+2007 thin space
    replace(dmsa, "\xe2\x80\x8a", '\0'); // U+200a hair space
    replace(dmsa, "\xe2\x80\x8b", '\0'); // U+200b invisible space
    replace(dmsa, "\xe2\x80\xaf", '\0'); // U+202f narrow space
    replace(dmsa, "\xe2\x81\xa3", '\0'); // U+2063 invisible separator

    replace(dmsa, "\xb0",         'd' ); // 0xb0 bare degree symbol
    replace(dmsa, "\xba",         'd' ); // 0xba bare alt symbol
    replace(dmsa, "*",            'd' ); // GRiD symbol for degree
    replace(dmsa, "`",            '\''); // grave accent
    replace(dmsa, "\xb4",         '\''); // 0xb4 bare acute accent
    // Don't implement these alternatives; they are only relevant for cgi-bin
    // replace(dmsa, "\x91",      '\''); // 0x91 ext ASCII left single quote
    // replace(dmsa, "\x92",      '\''); // 0x92 ext ASCII right single quote
    // replace(dmsa, "\x93",      '"' ); // 0x93 ext ASCII left double quote
    // replace(dmsa, "\x94",      '"' ); // 0x94 ext ASCII right double quote
    // replace(dmsa, "\x96",      '-' ); // 0x96 ext ASCII en dash
    // replace(dmsa, "\x97",      '-' ); // 0x97 ext ASCII em dash
    replace(dmsa, "\xa0",         '\0'); // 0xa0 bare non-breaking space
    replace(dmsa, "''",           '"' ); // '' -> "
    string::size_type
      beg = 0,
      end = unsigned(dmsa.size());
    while (beg < end && isspace(dmsa[beg]))
      ++beg;
    while (beg < end && isspace(dmsa[end - 1]))
      --end;
    // The trimmed string in [beg, end)
    real v = -0.0;              // So "-0" returns -0.0
    int i = 0;
    flag ind1 = NONE;
    // p is pointer to the next piece that needs decoding
    for (string::size_type p = beg, pb; p < end; p = pb, ++i) {
      string::size_type pa = p;
      // Skip over initial hemisphere letter (for i == 0)
      if (i == 0 && Utility::lookup(hemispheres_, dmsa[pa]) >= 0)
        ++pa;
      // Skip over initial sign (checking for it if i == 0)
      if (i > 0 || (pa < end && Utility::lookup(signs_, dmsa[pa]) >= 0))
        ++pa;
      // Find next sign
      pb = min(dmsa.find_first_of(signs_, pa), end);
      flag ind2 = NONE;
      v += InternalDecode(dmsa.substr(p, pb - p), ind2);
      if (ind1 == NONE)
        ind1 = ind2;
      else if (!(ind2 == NONE || ind1 == ind2))
        throw GeographicErr("Incompatible hemisphere specifier in " +
                            dmsa.substr(beg, pb - beg));
    }
    if (i == 0)
      throw GeographicErr("Empty or incomplete DMS string " +
                          dmsa.substr(beg, end - beg));
    ind = ind1;
    return v;
  }

  Math::real DMS::InternalDecode(const string& dmsa, flag& ind) {
    string errormsg;
    do {                       // Executed once (provides the ability to break)
      int sign = 1;
      unsigned
        beg = 0,
        end = unsigned(dmsa.size());
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
            errormsg = "Repeated " + string(components_[k]) +
              " component in " + dmsa.substr(beg, end - beg);
            break;
          } else if (unsigned(k) < npiece) {
            errormsg = string(components_[k]) + " component follows "
              + string(components_[npiece - 1]) + " component in "
              + dmsa.substr(beg, end - beg);
            break;
          }
          if (ncurrent == 0) {
            errormsg = "Missing numbers in " + string(components_[k]) +
              " component of " + dmsa.substr(beg, end - beg);
            break;
          }
          if (digcount > 0) {
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
        if (digcount > 0) {
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
      if (ipieces[1] >= Math::dm || fpieces[1] > Math::dm ) {
        errormsg = "Minutes " + Utility::str(fpieces[1])
          + " not in range [0, " + to_string(Math::dm) + ")";
        break;
      }
      if (ipieces[2] >= Math::ms || fpieces[2] > Math::ms) {
        errormsg = "Seconds " + Utility::str(fpieces[2])
          + " not in range [0, " + to_string(Math::ms) + ")";
        break;
      }
      ind = ind1;
      // Assume check on range of result is made by calling routine (which
      // might be able to offer a better diagnostic).
      return real(sign) *
        ( fpieces[2] != 0 ?
          (Math::ms*(Math::dm*fpieces[0] + fpieces[1]) + fpieces[2])/Math::ds :
          ( fpieces[1] != 0 ?
            (Math::dm*fpieces[0] + fpieces[1]) / Math::dm : fpieces[0] ) );
    } while (false);
    real val = Utility::nummatch<real>(dmsa);
    if (val == 0)
      throw GeographicErr(errormsg);
    else
      ind = NONE;
    return val;
  }

  void DMS::DecodeLatLon(const string& stra, const string& strb,
                         real& lat, real& lon,
                         bool longfirst) {
    real a, b;
    flag ia, ib;
    a = Decode(stra, ia);
    b = Decode(strb, ib);
    if (ia == NONE && ib == NONE) {
      // Default to lat, long unless longfirst
      ia = longfirst ? LONGITUDE : LATITUDE;
      ib = longfirst ? LATITUDE : LONGITUDE;
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
    if (fabs(lat1) > Math::qd)
      throw GeographicErr("Latitude " + Utility::str(lat1)
                          + "d not in [-" + to_string(Math::qd)
                          + "d, " + to_string(Math::qd) + "d]");
    lat = lat1;
    lon = lon1;
  }

  Math::real DMS::DecodeAngle(const string& angstr) {
    flag ind;
    real ang = Decode(angstr, ind);
    if (ind != NONE)
      throw GeographicErr("Arc angle " + angstr
                          + " includes a hemisphere, N/E/W/S");
    return ang;
  }

  Math::real DMS::DecodeAzimuth(const string& azistr) {
    flag ind;
    real azi = Decode(azistr, ind);
    if (ind == LATITUDE)
      throw GeographicErr("Azimuth " + azistr
                          + " has a latitude hemisphere, N/S");
    return Math::AngNormalize(azi);
  }

  string DMS::Encode(real angle, component trailing, unsigned prec, flag ind,
                     char dmssep) {
    // Assume check on range of input angle has been made by calling
    // routine (which might be able to offer a better diagnostic).
    if (!isfinite(angle))
      return angle < 0 ? string("-inf") :
        (angle > 0 ? string("inf") : string("nan"));

    // 15 - 2 * trailing = ceiling(log10(2^53/90/60^trailing)).
    // This suffices to give full real precision for numbers in [-90,90]
    prec = min(15 + Math::extra_digits() - 2 * unsigned(trailing), prec);
    real scale = trailing == MINUTE ? Math::dm :
      (trailing == SECOND ? Math::ds : 1);
    if (ind == AZIMUTH) {
      angle = Math::AngNormalize(angle);
      // Only angles strictly less than 0 can become 360; since +/-180 are
      // folded together, we convert -0 to +0 (instead of 360).
      if (angle < 0)
        angle += Math::td;
      else
        angle = Math::real(0) + angle;
    }
    int sign = signbit(angle) ? -1 : 1;
    angle *= sign;

    // Break off integer part to preserve precision and avoid overflow in
    // manipulation of fractional part for MINUTE and SECOND
    real
      idegree = trailing == DEGREE ? 0 : floor(angle),
      fdegree = (angle - idegree) * scale;
    string s = Utility::str(fdegree, prec), degree, minute, second;
    switch (trailing) {
    case DEGREE:
      degree = s;
      break;
    default:                    // case MINUTE: case SECOND:
      string::size_type p = s.find_first_of('.');
      long long i;
      if (p == 0)
        i = 0;
      else {
        i = stoll(s);
        if (p == string::npos)
          s.clear();
        else
          s = s.substr(p);
      }
      // Now i in [0,Math::dm] or [0,Math::ds] for MINUTE/DEGREE
      switch (trailing) {
      case MINUTE:
        minute = to_string(i % Math::dm) + s; i /= Math::dm;
        degree = Utility::str(i + idegree, 0); // no overflow since i in [0,1]
        break;
      default:                  // case SECOND:
        second = to_string(i % Math::ms) + s; i /= Math::ms;
        minute = to_string(i % Math::dm)    ; i /= Math::dm;
        degree = Utility::str(i + idegree, 0); // no overflow since i in [0,1]
        break;
      }
      break;
    }
    // No glue together degree+minute+second with
    // sign + zero-fill + delimiters + hemisphere
    ostringstream str;
    if (prec) ++prec;           // Extra width for decimal point
    if (ind == NONE && sign < 0)
      str << '-';
    str << setfill('0');
    switch (trailing) {
    case DEGREE:
      if (ind != NONE)
        str << setw(1 + min(int(ind), 2) + prec);
      str << degree;
      // Don't include degree designator (d) if it is the trailing component.
      break;
    case MINUTE:
      if (ind != NONE)
        str << setw(1 + min(int(ind), 2));
      str << degree << (dmssep ? dmssep : char(tolower(dmsindicators_[0])))
          << setw(2 + prec) << minute;
      if (!dmssep)
        str << char(tolower(dmsindicators_[1]));
      break;
    default:                    // case SECOND:
      if (ind != NONE)
        str << setw(1 + min(int(ind), 2));
      str << degree << (dmssep ? dmssep : char(tolower(dmsindicators_[0])))
          << setw(2)
          << minute << (dmssep ? dmssep : char(tolower(dmsindicators_[1])))
          << setw(2 + prec) << second;
      if (!dmssep)
        str << char(tolower(dmsindicators_[2]));
      break;
    }
    if (ind != NONE && ind != AZIMUTH)
      str << hemispheres_[(ind == LATITUDE ? 0 : 2) + (sign < 0 ? 0 : 1)];
    return str.str();
  }

} // namespace GeographicLib
