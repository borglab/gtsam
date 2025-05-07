/**
 * \file Utility.cpp
 * \brief Implementation for GeographicLib::Utility class
 *
 * Copyright (c) Charles Karney (2011-2022) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <cstdlib>
#include <GeographicLib/Utility.hpp>

#if defined(_MSC_VER)
// Squelch warnings about unsafe use of getenv
#  pragma warning (disable: 4996)
#endif

namespace GeographicLib {

  using namespace std;

  int Utility::day(int y, int m, int d) {
    // Convert from date to sequential day and vice versa
    //
    // Here is some code to convert a date to sequential day and vice
    // versa. The sequential day is numbered so that January 1, 1 AD is day 1
    // (a Saturday). So this is offset from the "Julian" day which starts the
    // numbering with 4713 BC.
    //
    // This is inspired by a talk by John Conway at the John von Neumann
    // National Supercomputer Center when he described his Doomsday algorithm
    // for figuring the day of the week. The code avoids explicitly doing ifs
    // (except for the decision of whether to use the Julian or Gregorian
    // calendar). Instead the equivalent result is achieved using integer
    // arithmetic. I got this idea from the routine for the day of the week
    // in MACLisp (I believe that that routine was written by Guy Steele).
    //
    // There are three issues to take care of
    //
    // 1. the rules for leap years,
    // 2. the inconvenient placement of leap days at the end of February,
    // 3. the irregular pattern of month lengths.
    //
    // We deal with these as follows:
    //
    // 1. Leap years are given by simple rules which are straightforward to
    // accommodate.
    //
    // 2. We simplify the calculations by moving January and February to the
    // previous year. Here we internally number the months March–December,
    // January, February as 0–9, 10, 11.
    //
    // 3. The pattern of month lengths from March through January is regular
    // with a 5-month period—31, 30, 31, 30, 31, 31, 30, 31, 30, 31, 31. The
    // 5-month period is 153 days long. Since February is now at the end of
    // the year, we don't need to include its length in this part of the
    // calculation.
    bool greg = gregorian(y, m, d);
    y += (m + 9) / 12 - 1; // Move Jan and Feb to previous year,
    m = (m + 9) % 12;      // making March month 0.
    return
      (1461 * y) / 4 // Julian years converted to days.  Julian year is 365 +
      // 1/4 = 1461/4 days.
      // Gregorian leap year corrections.  The 2 offset with respect to the
      // Julian calendar synchronizes the vernal equinox with that at the
      // time of the Council of Nicea (325 AD).
      + (greg ? (y / 100) / 4 - (y / 100) + 2 : 0)
      + (153 * m + 2) / 5     // The zero-based start of the m'th month
      + d - 1                 // The zero-based day
      - 305; // The number of days between March 1 and December 31.
    // This makes 0001-01-01 day 1
  }

  int Utility::day(int y, int m, int d, bool check) {
    int s = day(y, m, d);
    if (!check)
      return s;
    int y1, m1, d1;
    date(s, y1, m1, d1);
    if (!(s > 0 && y == y1 && m == m1 && d == d1))
      throw GeographicErr("Invalid date " +
                          str(y) + "-" + str(m) + "-" + str(d)
                          + (s > 0 ? "; use " +
                             str(y1) + "-" + str(m1) + "-" + str(d1) :
                             " before 0001-01-01"));
    return s;
  }

  void Utility::date(int s, int& y, int& m, int& d) {
    int c = 0;
    bool greg = gregorian(s);
    s += 305;                 // s = 0 on March 1, 1BC
    if (greg) {
      s -= 2;                 // The 2 day Gregorian offset
      // Determine century with the Gregorian rules for leap years.  The
      // Gregorian year is 365 + 1/4 - 1/100 + 1/400 = 146097/400 days.
      c = (4 * s + 3) / 146097;
      s -= (c * 146097) / 4;  // s = 0 at beginning of century
    }
    y = (4 * s + 3) / 1461;   // Determine the year using Julian rules.
    s -= (1461 * y) / 4;      // s = 0 at start of year, i.e., March 1
    y += c * 100;             // Assemble full year
    m = (5 * s + 2) / 153;    // Determine the month
    s -= (153 * m + 2) / 5;   // s = 0 at beginning of month
    d = s + 1;                // Determine day of month
    y += (m + 2) / 12;        // Move Jan and Feb back to original year
    m = (m + 2) % 12 + 1;     // Renumber the months so January = 1
  }

  void Utility::date(const std::string& s, int& y, int& m, int& d) {
    if (s == "now") {
      time_t t = time(0);
      struct tm* now = gmtime(&t);
      y = now->tm_year + 1900;
      m = now->tm_mon + 1;
      d = now->tm_mday;
      return;
    }
    int y1, m1 = 1, d1 = 1;
    const char* digits = "0123456789";
    string::size_type p1 = s.find_first_not_of(digits);
    if (p1 == string::npos)
      y1 = val<int>(s);
    else if (s[p1] != '-')
      throw GeographicErr("Delimiter not hyphen in date " + s);
    else if (p1 == 0)
      throw GeographicErr("Empty year field in date " + s);
    else {
      y1 = val<int>(s.substr(0, p1));
      if (++p1 == s.size())
        throw GeographicErr("Empty month field in date " + s);
      string::size_type p2 = s.find_first_not_of(digits, p1);
      if (p2 == string::npos)
        m1 = val<int>(s.substr(p1));
      else if (s[p2] != '-')
        throw GeographicErr("Delimiter not hyphen in date " + s);
      else if (p2 == p1)
        throw GeographicErr("Empty month field in date " + s);
      else {
        m1 = val<int>(s.substr(p1, p2 - p1));
        if (++p2 == s.size())
          throw GeographicErr("Empty day field in date " + s);
        d1 = val<int>(s.substr(p2));
      }
    }
    y = y1; m = m1; d = d1;
  }

  std::string Utility::trim(const std::string& s) {
    unsigned
      beg = 0,
      end = unsigned(s.size());
    while (beg < end && isspace(s[beg]))
      ++beg;
    while (beg < end && isspace(s[end - 1]))
      --end;
    return string(s, beg, end-beg);
  }

  int Utility::lookup(const std::string& s, char c) {
    string::size_type r = s.find(char(toupper(c)));
    return r == string::npos ? -1 : int(r);
  }

  int Utility::lookup(const char* s, char c) {
    const char* p = strchr(s, toupper(c));
    return p != NULL ? int(p - s) : -1;
  }

  bool Utility::ParseLine(const std::string& line,
                          std::string& key, std::string& value,
                          char equals, char comment) {
    key.clear(); value.clear();
    string::size_type n = comment ? line.find(comment) : line.size();
    string linea = trim(line.substr(0, n));
    if (linea.empty()) return false;
    n = equals ? linea.find(equals) : linea.find_first_of(" \t\n\v\f\r");
    key = trim(linea.substr(0, n));
    if (key.empty()) return false;
    if (n != string::npos) value = trim(linea.substr(n + 1));
    return true;
  }

  int Utility::set_digits(int ndigits) {
#if GEOGRAPHICLIB_PRECISION == 5
    if (ndigits <= 0) {
      char* digitenv = getenv("GEOGRAPHICLIB_DIGITS");
      if (digitenv)
        ndigits = strtol(digitenv, NULL, 0);
      if (ndigits <= 0)
        ndigits = 256;
    }
#endif
    return Math::set_digits(ndigits);
  }

} // namespace GeographicLib
