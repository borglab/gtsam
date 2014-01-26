/**
 * \file Utility.hpp
 * \brief Header for GeographicLib::Utility class
 *
 * Copyright (c) Charles Karney (2011-2012) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * http://geographiclib.sourceforge.net/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_UTILITY_HPP)
#define GEOGRAPHICLIB_UTILITY_HPP 1

#include <GeographicLib/Constants.hpp>
#include <iomanip>
#include <vector>
#include <sstream>
#include <cctype>

#if defined(_MSC_VER)
// Squelch warnings about constant conditional expressions
#  pragma warning (push)
#  pragma warning (disable: 4127)
#endif

namespace GeographicLib {

  /**
   * \brief Some utility routines for %GeographicLib
   *
   * Example of use:
   * \include example-Utility.cpp
   **********************************************************************/
  class GEOGRAPHICLIB_EXPORT Utility {
  private:
    static bool gregorian(int y, int m, int d) {
      // The original cut over to the Gregorian calendar in Pope Gregory XIII's
      // time had 1582-10-04 followed by 1582-10-15. Here we implement the
      // switch over used by the English-speaking world where 1752-09-02 was
      // followed by 1752-09-14. We also assume that the year always begins
      // with January 1, whereas in reality it often was reckoned to begin in
      // March.
      return 100 * (100 * y + m) + d >= 17520914; // or 15821004
    }
    static bool gregorian(int s) {
      return s >= 639799;       // 1752-09-14
    }
  public:

    /**
     * Convert a date to the day numbering sequentially starting with
     * 0001-01-01 as day 1.
     *
     * @param[in] y the year (must be positive).
     * @param[in] m the month, Jan = 1, etc. (must be positive).  Default = 1.
     * @param[in] d the day of the month (must be positive).  Default = 1.
     * @return the sequential day number.
     **********************************************************************/
    static int day(int y, int m = 1, int d = 1) throw() {
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
        // Julian calendar synchronizes the vernal equinox with that at the time
        // of the Council of Nicea (325 AD).
        + (greg ? (y / 100) / 4 - (y / 100) + 2 : 0)
        + (153 * m + 2) / 5     // The zero-based start of the m'th month
        + d - 1                 // The zero-based day
        - 305; // The number of days between March 1 and December 31.
               // This makes 0001-01-01 day 1
    }

    /**
     * Convert a date to the day numbering sequentially starting with
     * 0001-01-01 as day 1.
     *
     * @param[in] y the year (must be positive).
     * @param[in] m the month, Jan = 1, etc. (must be positive).  Default = 1.
     * @param[in] d the day of the month (must be positive).  Default = 1.
     * @param[in] check whether to check the date.
     * @exception GeographicErr if the date is invalid and \e check is true.
     * @return the sequential day number.
     **********************************************************************/
    static int day(int y, int m, int d, bool check) {
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

    /**
     * Given a day (counting from 0001-01-01 as day 1), return the date.
     *
     * @param[in] s the sequential day number (must be positive)
     * @param[out] y the year.
     * @param[out] m the month, Jan = 1, etc.
     * @param[out] d the day of the month.
     **********************************************************************/
    static void date(int s, int& y, int& m, int& d) throw() {
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

    /**
     * Given a date as a string in the format yyyy, yyyy-mm, or yyyy-mm-dd,
     * return the numeric values for the year, month, and day.  No checking is
     * done on these values.
     *
     * @param[in] s the date in string format.
     * @param[out] y the year.
     * @param[out] m the month, Jan = 1, etc.
     * @param[out] d the day of the month.
     * @exception GeographicErr is \e s is malformed.
     **********************************************************************/
    static void date(const std::string& s, int& y, int& m, int& d) {
      int y1, m1 = 1, d1 = 1;
      const char* digits = "0123456789";
      std::string::size_type p1 = s.find_first_not_of(digits);
      if (p1 == std::string::npos)
        y1 = num<int>(s);
      else if (s[p1] != '-')
        throw GeographicErr("Delimiter not hyphen in date " + s);
      else if (p1 == 0)
        throw GeographicErr("Empty year field in date " + s);
      else {
        y1 = num<int>(s.substr(0, p1));
        if (++p1 == s.size())
          throw GeographicErr("Empty month field in date " + s);
        std::string::size_type p2 = s.find_first_not_of(digits, p1);
        if (p2 == std::string::npos)
          m1 = num<int>(s.substr(p1));
        else if (s[p2] != '-')
          throw GeographicErr("Delimiter not hyphen in date " + s);
        else if (p2 == p1)
          throw GeographicErr("Empty month field in date " + s);
        else {
          m1 = num<int>(s.substr(p1, p2 - p1));
          if (++p2 == s.size())
            throw GeographicErr("Empty day field in date " + s);
          d1 = num<int>(s.substr(p2));
        }
      }
      y = y1; m = m1; d = d1;
    }

    /**
     * Given the date, return the day of the week.
     *
     * @param[in] y the year (must be positive).
     * @param[in] m the month, Jan = 1, etc. (must be positive).
     * @param[in] d the day of the month (must be positive).
     * @return the day of the week with Sunday, Monday--Saturday = 0,
     *   1--6.
     **********************************************************************/
    static int dow(int y, int m, int d) throw() { return dow(day(y, m, d)); }

    /**
     * Given the sequential day, return the day of the week.
     *
     * @param[in] s the sequential day (must be positive).
     * @return the day of the week with Sunday, Monday--Saturday = 0,
     *   1--6.
     **********************************************************************/
    static int dow(int s) throw() {
      return (s + 5) % 7;  // The 5 offset makes day 1 (0001-01-01) a Saturday.
    }

    /**
     * Convert a string representing a date to a fractional year.
     *
     * @tparam T the type of the argument.
     * @param[in] s the string to be converted.
     * @exception GeographicErr if \e s can't be interpreted as a date.
     * @return the fractional year.
     *
     * The string is first read as an ordinary number (e.g., 2010 or 2012.5);
     * if this is successful, the value is returned.  Otherwise the string
     * should be of the form yyyy-mm or yyyy-mm-dd and this is converted to a
     * number with 2010-01-01 giving 2010.0 and 2012-07-03 giving 2012.5.
     **********************************************************************/
    template<typename T> static T fractionalyear(const std::string& s) {
      try {
        return num<T>(s);
      }
      catch (const std::exception&) {
      }
      int y, m, d;
      date(s, y, m, d);
      int t = day(y, m, d, true);
      return T(y) + T(t - day(y)) / T(day(y + 1) - day(y));
    }

    /**
     * Convert a object of type T to a string.
     *
     * @tparam T the type of the argument.
     * @param[in] x the value to be converted.
     * @param[in] p the precision used (default &minus;1).
     * @exception std::bad_alloc if memory for the string can't be allocated.
     * @return the string representation.
     *
     * If \e p &ge; 0, then the number fixed format is used with p bits of
     * precision.  With p < 0, there is no manipulation of the format.
     **********************************************************************/
    template<typename T> static std::string str(T x, int p = -1) {
      if (!std::numeric_limits<T>::is_integer && !Math::isfinite<T>(x))
        return x < 0 ? std::string("-inf") :
          (x > 0 ? std::string("inf") : std::string("nan"));
      std::ostringstream s;
      if (p >= 0) s << std::fixed << std::setprecision(p);
      s << x; return s.str();
    }

    /**
     * Convert a string to an object of type T.
     *
     * @tparam T the type of the return value.
     * @param[in] s the string to be converted.
     * @exception GeographicErr is \e s is not readable as a T.
     * @return object of type T
     **********************************************************************/
    template<typename T> static T num(const std::string& s) {
      T x;
      std::string errmsg;
      do {                     // Executed once (provides the ability to break)
        std::istringstream is(s);
        if (!(is >> x)) {
          errmsg = "Cannot decode " + s;
          break;
        }
        int pos = int(is.tellg()); // Returns -1 at end of string?
        if (!(pos < 0 || pos == int(s.size()))) {
          errmsg = "Extra text " + s.substr(pos) + " at end of " + s;
          break;
        }
        return x;
      } while (false);
      x = std::numeric_limits<T>::is_integer ? 0 : nummatch<T>(s);
      if (x == 0)
        throw GeographicErr(errmsg);
      return x;
    }

    /**
     * Match "nan" and "inf" (and variants thereof) in a string.
     *
     * @tparam T the type of the return value.
     * @param[in] s the string to be matched.
     * @return appropriate special value (&plusmn;&infin;, nan) or 0 if none is
     *   found.
     **********************************************************************/
    template<typename T> static T nummatch(const std::string& s) {
      if (s.length() < 3)
        return 0;
      std::string t;
      t.resize(s.length());
      std::transform(s.begin(), s.end(), t.begin(), (int(*)(int))std::toupper);
      for (size_t i = s.length(); i--;)
        t[i] = char(std::toupper(s[i]));
      int sign = t[0] == '-' ? -1 : 1;
      std::string::size_type p0 = t[0] == '-' || t[0] == '+' ? 1 : 0;
      std::string::size_type p1 = t.find_last_not_of('0');
      if (p1 == std::string::npos || p1 + 1 < p0 + 3)
        return 0;
      // Strip off sign and trailing 0s
      t = t.substr(p0, p1 + 1 - p0);  // Length at least 3
      if (t == "NAN" || t == "1.#QNAN" || t == "1.#SNAN" || t == "1.#IND" ||
          t == "1.#R")
        return Math::NaN<T>();
      else if (t == "INF" || t == "1.#INF")
        return sign * Math::infinity<T>();
      return 0;
    }

    /**
     * Read a simple fraction, e.g., 3/4, from a string to an object of type T.
     *
     * @tparam T the type of the return value.
     * @param[in] s the string to be converted.
     * @exception GeographicErr is \e s is not readable as a fraction of type T.
     * @return object of type T
     **********************************************************************/
    template<typename T> static T fract(const std::string& s) {
      std::string::size_type delim = s.find('/');
      return
        !(delim != std::string::npos && delim >= 1 && delim + 2 <= s.size()) ?
        num<T>(s) :
        // delim in [1, size() - 2]
        num<T>(s.substr(0, delim)) / num<T>(s.substr(delim + 1));
    }

    /**
     * Lookup up a character in a string.
     *
     * @param[in] s the string to be searched.
     * @param[in] c the character to look for.
     * @return the index of the first occurrence character in the string or
     *   &minus;1 is the character is not present.
     *
     * \e c is converted to upper case before search \e s.  Therefore, it is
     * intended that \e s should not contain any lower case letters.
     **********************************************************************/
    static int lookup(const std::string& s, char c) throw() {
      std::string::size_type r = s.find(char(toupper(c)));
      return r == std::string::npos ? -1 : int(r);
    }

    /**
     * Read data of type ExtT from a binary stream to an array of type IntT.
     * The data in the file is in (bigendp ? big : little)-endian format.
     *
     * @tparam ExtT the type of the objects in the binary stream (external).
     * @tparam IntT the type of the objects in the array (internal).
     * @tparam bigendp true if the external storage format is big-endian.
     * @param[in] str the input stream containing the data of type ExtT
     *   (external).
     * @param[out] array the output array of type IntT (internal).
     * @param[in] num the size of the array.
     * @exception GeographicErr if the data cannot be read.
     **********************************************************************/
    template<typename ExtT, typename IntT, bool bigendp>
      static inline void readarray(std::istream& str,
                                   IntT array[], size_t num) {
      if (sizeof(IntT) == sizeof(ExtT) &&
          std::numeric_limits<IntT>::is_integer ==
          std::numeric_limits<ExtT>::is_integer) {
        // Data is compatible (aside from the issue of endian-ness).
        str.read(reinterpret_cast<char *>(array), num * sizeof(ExtT));
        if (!str.good())
          throw GeographicErr("Failure reading data");
        if (bigendp != Math::bigendian) { // endian mismatch -> swap bytes
          for (size_t i = num; i--;)
            array[i] = Math::swab<IntT>(array[i]);
        }
      } else {
        const int bufsize = 1024; // read this many values at a time
        ExtT buffer[bufsize];     // temporary buffer
        int k = int(num);         // data values left to read
        int i = 0;                // index into output array
        while (k) {
          int n = (std::min)(k, bufsize);
          str.read(reinterpret_cast<char *>(buffer), n * sizeof(ExtT));
          if (!str.good())
            throw GeographicErr("Failure reading data");
          for (int j = 0; j < n; ++j)
            // fix endian-ness and cast to IntT
            array[i++] = IntT(bigendp == Math::bigendian ? buffer[j] :
                              Math::swab<ExtT>(buffer[j]));
          k -= n;
        }
      }
      return;
    }

    /**
     * Read data of type ExtT from a binary stream to a vector array of type
     * IntT.  The data in the file is in (bigendp ? big : little)-endian
     * format.
     *
     * @tparam ExtT the type of the objects in the binary stream (external).
     * @tparam IntT the type of the objects in the array (internal).
     * @tparam bigendp true if the external storage format is big-endian.
     * @param[in] str the input stream containing the data of type ExtT
     *   (external).
     * @param[out] array the output vector of type IntT (internal).
     * @exception GeographicErr if the data cannot be read.
     **********************************************************************/
    template<typename ExtT, typename IntT, bool bigendp>
      static inline void readarray(std::istream& str,
                                   std::vector<IntT>& array) {
      readarray<ExtT, IntT, bigendp>(str, &array[0], array.size());
    }

    /**
     * Write data in an array of type IntT as type ExtT to a binary stream.
     * The data in the file is in (bigendp ? big : little)-endian format.
     *
     * @tparam ExtT the type of the objects in the binary stream (external).
     * @tparam IntT the type of the objects in the array (internal).
     * @tparam bigendp true if the external storage format is big-endian.
     * @param[out] str the output stream for the data of type ExtT (external).
     * @param[in] array the input array of type IntT (internal).
     * @param[in] num the size of the array.
     * @exception GeographicErr if the data cannot be written.
     **********************************************************************/
    template<typename ExtT, typename IntT, bool bigendp>
      static inline void writearray(std::ostream& str,
                                   const IntT array[], size_t num) {
      if (sizeof(IntT) == sizeof(ExtT) &&
          std::numeric_limits<IntT>::is_integer ==
          std::numeric_limits<ExtT>::is_integer &&
          bigendp == Math::bigendian) {
        // Data is compatible (including endian-ness).
        str.write(reinterpret_cast<const char *>(array), num * sizeof(ExtT));
        if (!str.good())
          throw GeographicErr("Failure writing data");
      } else {
        const int bufsize = 1024; // write this many values at a time
        ExtT buffer[bufsize];     // temporary buffer
        int k = int(num);         // data values left to write
        int i = 0;                // index into output array
        while (k) {
          int n = (std::min)(k, bufsize);
          for (int j = 0; j < n; ++j)
            // cast to ExtT and fix endian-ness
            buffer[j] = bigendp == Math::bigendian ? ExtT(array[i++]) :
              Math::swab<ExtT>(ExtT(array[i++]));
          str.write(reinterpret_cast<const char *>(buffer), n * sizeof(ExtT));
          if (!str.good())
            throw GeographicErr("Failure writing data");
          k -= n;
        }
      }
      return;
    }

    /**
     * Write data in an array of type IntT as type ExtT to a binary stream.
     * The data in the file is in (bigendp ? big : little)-endian format.
     *
     * @tparam ExtT the type of the objects in the binary stream (external).
     * @tparam IntT the type of the objects in the array (internal).
     * @tparam bigendp true if the external storage format is big-endian.
     * @param[out] str the output stream for the data of type ExtT (external).
     * @param[in] array the input vector of type IntT (internal).
     * @exception GeographicErr if the data cannot be written.
     **********************************************************************/
    template<typename ExtT, typename IntT, bool bigendp>
      static inline void writearray(std::ostream& str,
                                   std::vector<IntT>& array) {
      writearray<ExtT, IntT, bigendp>(str, &array[0], array.size());
    }

    /**
     * Parse a KEY VALUE line.
     *
     * @param[in] line the input line.
     * @param[out] key the key.
     * @param[out] val the value.
     * @exception std::bad_alloc if memory for the internal strings can't be
     *   allocated.
     * @return whether a key was found.
     *
     * A # character and everything after it are discarded.  If the results is
     * just white space, the routine returns false (and \e key and \e val are
     * not set).  Otherwise the first token is taken to be the key and the rest
     * of the line (trimmed of leading and trailing white space) is the value.
     **********************************************************************/
    static bool ParseLine(const std::string& line,
                          std::string& key, std::string& val);

  };

} // namespace GeographicLib

#if defined(_MSC_VER)
#  pragma warning (pop)
#endif

#endif  // GEOGRAPHICLIB_UTILITY_HPP
