/**
 * \file Utility.hpp
 * \brief Header for GeographicLib::Utility class
 *
 * Copyright (c) Charles Karney (2011-2024) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#if !defined(GEOGRAPHICLIB_UTILITY_HPP)
#define GEOGRAPHICLIB_UTILITY_HPP 1

#include <GeographicLib/Constants.hpp>
#include <iomanip>
#include <vector>
#include <sstream>
#include <cctype>
#include <ctime>
#include <cstring>

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
      return 100 * (100 * y + m) + d >= 17520914; // or 15821015
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
    static int day(int y, int m = 1, int d = 1);

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
    static int day(int y, int m, int d, bool check);

    /**
     * Given a day (counting from 0001-01-01 as day 1), return the date.
     *
     * @param[in] s the sequential day number (must be positive)
     * @param[out] y the year.
     * @param[out] m the month, Jan = 1, etc.
     * @param[out] d the day of the month.
     **********************************************************************/
    static void date(int s, int& y, int& m, int& d);

    /**
     * Given a date as a string in the format yyyy, yyyy-mm, or yyyy-mm-dd,
     * return the numeric values for the year, month, and day.  No checking is
     * done on these values.  The string "now" is interpreted as the present
     * date (in UTC).
     *
     * @param[in] s the date in string format.
     * @param[out] y the year.
     * @param[out] m the month, Jan = 1, etc.
     * @param[out] d the day of the month.
     * @exception GeographicErr is \e s is malformed.
     **********************************************************************/
    static void date(const std::string& s, int& y, int& m, int& d);

    /**
     * Given the date, return the day of the week.
     *
     * @param[in] y the year (must be positive).
     * @param[in] m the month, Jan = 1, etc. (must be positive).
     * @param[in] d the day of the month (must be positive).
     * @return the day of the week with Sunday, Monday--Saturday = 0,
     *   1--6.
     **********************************************************************/
    static int dow(int y, int m, int d) { return dow(day(y, m, d)); }

    /**
     * Given the sequential day, return the day of the week.
     *
     * @param[in] s the sequential day (must be positive).
     * @return the day of the week with Sunday, Monday--Saturday = 0,
     *   1--6.
     **********************************************************************/
    static int dow(int s) {
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
     * number with 2010-01-01 giving 2010.0 and 2012-07-03 giving 2012.5.  The
     * string "now" is interpreted as the present date.
     **********************************************************************/
    template<typename T> static T fractionalyear(const std::string& s) {
      try {
        return val<T>(s);
      }
      catch (const std::exception&) {}
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
     * If \e p &ge; 0, then the number fixed format is used with \e p bits of
     * precision.  With \e p < 0, there is no manipulation of the format,
     * except that <code>boolalpha</code> is used to represent bools as "true"
     * and "false".  There is an overload of this function if T is Math::real;
     * this deals with inf and nan.
     **********************************************************************/
    template<typename T> static std::string str(T x, int p = -1) {
      std::ostringstream s;
      if (p >= 0) s << std::fixed << std::setprecision(p);
      s << std::boolalpha << x; return s.str();
    }

    /**
     * Trim the white space from the beginning and end of a string.
     *
     * @param[in] s the string to be trimmed
     * @return the trimmed string
     **********************************************************************/
    static std::string trim(const std::string& s);

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
    static int lookup(const std::string& s, char c);

    /**
     * Lookup up a character in a char*.
     *
     * @param[in] s the char* string to be searched.
     * @param[in] c the character to look for.
     * @return the index of the first occurrence character in the string or
     *   &minus;1 is the character is not present.
     *
     * \e c is converted to upper case before search \e s.  Therefore, it is
     * intended that \e s should not contain any lower case letters.
     **********************************************************************/
    static int lookup(const char* s, char c);

    /**
     * Convert a string to type T.
     *
     * @tparam T the type of the return value.
     * @param[in] s the string to be converted.
     * @exception GeographicErr is \e s is not readable as a T.
     * @return object of type T.
     *
     * White space at the beginning and end of \e s is ignored.
     *
     * Special handling is provided for some types.
     *
     * If T is a floating point type, then inf and nan are recognized.
     *
     * If T is bool, then \e s should either be string a representing 0 (false)
     * or 1 (true) or one of the strings
     * - "false", "f", "nil", "no", "n", "off", or "" meaning false,
     * - "true", "t", "yes", "y", or "on" meaning true;
     * .
     * case is ignored.
     *
     * If T is std::string, then \e s is returned (with the white space at the
     * beginning and end removed).
     **********************************************************************/
    template<typename T> static T val(const std::string& s) {
      // If T is bool, then the specialization val<bool>() defined below is
      // used.
      T x;
      std::string errmsg, t(trim(s));
      do {                     // Executed once (provides the ability to break)
        std::istringstream is(t);
        if (!(is >> x)) {
          errmsg = "Cannot decode " + t;
          break;
        }
        int pos = int(is.tellg()); // Returns -1 at end of string?
        if (!(pos < 0 || pos == int(t.size()))) {
          errmsg = "Extra text " + t.substr(pos) + " at end of " + t;
          break;
        }
        return x;
      } while (false);
      x = std::numeric_limits<T>::is_integer ? 0 : nummatch<T>(t);
      if (x == 0)
        throw GeographicErr(errmsg);
      return x;
    }

    /**
     * Match "nan" and "inf" (and variants thereof) in a string.
     *
     * @tparam T the type of the return value (this should be a floating point
     *   type).
     * @param[in] s the string to be matched.
     * @return appropriate special value (&plusmn;&infin;, nan) or 0 if none is
     *   found.
     *
     * White space is not allowed at the beginning or end of \e s.
     **********************************************************************/
    template<typename T> static T nummatch(const std::string& s) {
      if (s.length() < 3)
        return 0;
      std::string t(s);
      for (std::string::iterator p = t.begin(); p != t.end(); ++p)
        *p = char(std::toupper(*p));
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
      else if (t == "INF" || t == "1.#INF" || t == "INFINITY")
        return sign * Math::infinity<T>();
      return 0;
    }

    /**
     * Read a simple fraction, e.g., 3/4, from a string to an object of type T.
     *
     * @tparam T the type of the return value.
     * @param[in] s the string to be converted.
     * @exception GeographicErr is \e s is not readable as a fraction of type
     *   T.
     * @return object of type T
     *
     * \note The msys shell under Windows converts arguments which look like
     * pathnames into their Windows equivalents.  As a result the argument
     * "-1/300" gets mangled into something unrecognizable.  A workaround is to
     * use a floating point number in the numerator, i.e., "-1.0/300".  (Recent
     * versions of the msys shell appear \e not to have this problem.)
     **********************************************************************/
    template<typename T> static T fract(const std::string& s) {
      std::string::size_type delim = s.find('/');
      return
        !(delim != std::string::npos && delim >= 1 && delim + 2 <= s.size()) ?
        val<T>(s) :
        // delim in [1, size() - 2]
        val<T>(s.substr(0, delim)) / val<T>(s.substr(delim + 1));
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
      static void readarray(std::istream& str, IntT array[], size_t num) {
#if GEOGRAPHICLIB_PRECISION < 4
      // for C++17 use if constexpr
      if (sizeof(IntT) == sizeof(ExtT) &&
          std::numeric_limits<IntT>::is_integer ==
          std::numeric_limits<ExtT>::is_integer)
        {
          // Data is compatible (aside from the issue of endian-ness).
          str.read(reinterpret_cast<char*>(array), num * sizeof(ExtT));
          if (!str.good())
            throw GeographicErr("Failure reading data");
          // for C++17 use if constexpr
          if (bigendp != Math::bigendian) { // endian mismatch -> swap bytes
            for (size_t i = num; i--;)
              array[i] = Math::swab<IntT>(array[i]);
          }
        }
      else
#endif
        {
          const int bufsize = 1024; // read this many values at a time
          ExtT buffer[bufsize];     // temporary buffer
          int k = int(num);         // data values left to read
          int i = 0;                // index into output array
          while (k) {
            int n = (std::min)(k, bufsize);
            str.read(reinterpret_cast<char*>(buffer), n * sizeof(ExtT));
            if (!str.good())
              throw GeographicErr("Failure reading data");
            for (int j = 0; j < n; ++j) {
              // fix endian-ness
              ExtT x = bigendp == Math::bigendian ? buffer[j] :
                Math::swab<ExtT>(buffer[j]);
#if GEOGRAPHICLIB_PRECISION > 2
              // for C++17 use if constexpr folding in test of
              if (typeid(ExtT) == typeid(double) &&
                    typeid(IntT) == typeid(Math::real)) {
                // readarray is used to read in coefficient data rapidly.  Thus
                // 8.3n is stored in its IEEE double representation.  This is
                // fine is the working precision is double.  However, when
                // working at higher precision, how should be interpret the
                // constant 8.3 appearing in a published table?  Possibilities
                // are
                //
                // (a) treat this as an exact decimal number 83/10;
                //
                // (b) treat this as the approximate decimal representation of
                // an exact double precision number 2336242306698445/2^48 =
                // 8.300000000000000710542735760100185871124267578125
                //
                // Here use (a) if the number of significant digits in the
                // number is 15 or less.  Otherwise, we use (b).
                //
                // We implement this as follows.  Any double which can be
                // represented as a decimal number with precision 14 = digis10
                // - 1 (= 15 sig figs) is treated as an approximation to that
                // decimal number.  The high precision number is then obtained
                // by reading the decimal number at that precision.  Otherwise
                // the double is treated as exact.  The high precision number
                // is obtained by adding zeros in the binary fraction.
                //
                // N.B. printing with precision 14 = digis10 - 1 allows short
                // numbers to be represended with trailing zeros.  This isn't
                // necessarily the case with precision = digits10, e.g., 8.3
                // becomes 8.300000000000001
                //
                // This prescription doesn't exactly implement the method
                // proposed.  If the published table of numbers includes
                // 8.300000000000001, this will be interpreted as 8.3.  This
                // doesn't apply to any published magnetic or gravity data.
                // E.g., the coefficients for EGM96, resp. EGM2008, are given
                // with precision 11, resp. 14.
                //
                // This conversion of doubles to Math::real comes at a
                // substantial cost.  It adds about 14 s to the time it takes
                // to read the egm2008 gravity model for quad and mpfr
                // precisions.  This is acceptable, however, because high
                // precision is only used for benchmarking.
                std::ostringstream str;
                str << std::scientific
                    << std::setprecision(std::numeric_limits<ExtT>::digits10-1)
                    << x;
                if (val<ExtT>(str.str()) == x)
                  array[i++] = val<IntT>(str.str());
                else
                  array[i++] = IntT(x);
              } else {
                // Code for GEOGRAPHILIB_PRECISION > 2 but types not
                // double/real
                array[i++] = IntT(x);
              }
#else
              // Code for GEOGRAPHILIB_PRECISION <= 2
              array[i++] = IntT(x);
#endif
            }
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
      static void readarray(std::istream& str, std::vector<IntT>& array) {
      if (array.size() > 0)
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
      static void writearray(std::ostream& str, const IntT array[], size_t num)
    {
#if GEOGRAPHICLIB_PRECISION < 4
      if (sizeof(IntT) == sizeof(ExtT) &&
          std::numeric_limits<IntT>::is_integer ==
          std::numeric_limits<ExtT>::is_integer &&
          bigendp == Math::bigendian)
        {
          // Data is compatible (including endian-ness).
          str.write(reinterpret_cast<const char*>(array), num * sizeof(ExtT));
          if (!str.good())
            throw GeographicErr("Failure writing data");
        }
      else
#endif
        {
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
            str.write(reinterpret_cast<const char*>(buffer), n * sizeof(ExtT));
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
      static void writearray(std::ostream& str, std::vector<IntT>& array) {
      if (array.size() > 0)
        writearray<ExtT, IntT, bigendp>(str, &array[0], array.size());
    }

    /**
     * Parse a KEY [=] VALUE line.
     *
     * @param[in] line the input line.
     * @param[out] key the KEY.
     * @param[out] value the VALUE.
     * @param[in] equals character representing "equals" to separate KEY and
     *   VALUE, if NULL (the default) use first space character.
     * @param[in] comment character to use as the comment character; if
     *   non-NULL, this character and everything after it is discarded; default
     *   is '#'.
     * @exception std::bad_alloc if memory for the internal strings can't be
     *   allocated.
     * @return whether a key was found.
     *
     * The \e comment character (default is '#') and everything after it are
     * discarded and the result trimmed of leading and trailing white space.
     * Use the \e equals delimiter character (or, if it is NULL -- the default,
     * the first white space) to separate \e key and \e value.  \e key and \e
     * value are trimmed of leading and trailing white space.  If \e key is
     * empty, then \e value is set to "" and false is returned.
     **********************************************************************/
    static bool ParseLine(const std::string& line,
                          std::string& key, std::string& value,
                          char equals = '\0', char comment = '#');

    /**
     * Set the binary precision of a real number.
     *
     * @param[in] ndigits the number of bits of precision.  If ndigits is 0
     *   (the default), then determine the precision from the environment
     *   variable GEOGRAPHICLIB_DIGITS.  If this is undefined, use ndigits =
     *   256 (i.e., about 77 decimal digits).
     * @return the resulting number of bits of precision.
     *
     * This only has an effect when GEOGRAPHICLIB_PRECISION = 5.  The
     * precision should only be set once and before calls to any other
     * GeographicLib functions.  (Several functions, for example Math::pi(),
     * cache the return value in a static local variable.  The precision needs
     * to be set before a call to any such functions.)  In multi-threaded
     * applications, it is necessary also to set the precision in each thread
     * (see the example GeoidToGTX.cpp).
     *
     * \note Use Math::digits() to return the current precision in bits.
     **********************************************************************/
    static int set_digits(int ndigits = 0);

  };

  /**
   * The specialization of Utility::val<T>() for strings.
   *
   * @param[in] s the string to be converted.
   * @exception GeographicErr is \e s is not readable as a T.
   * @return the string trimmed of its whitespace.
   **********************************************************************/
  template<> inline std::string Utility::val<std::string>(const std::string& s)
  { return trim(s); }

  /**
   * The specialization of Utility::val<T>() for bools.
   *
   * @param[in] s the string to be converted.
   * @exception GeographicErr is \e s is not readable as a T.
   * @return boolean value.
   *
   * \e s should either be string a representing 0 (false)
   * or 1 (true) or one of the strings
   * - "false", "f", "nil", "no", "n", "off", or "" meaning false,
   * - "true", "t", "yes", "y", or "on" meaning true;
   * .
   * case is ignored.
   **********************************************************************/
  template<> inline bool Utility::val<bool>(const std::string& s) {
    std::string t(trim(s));
    if (t.empty()) return false;
    bool x;
    {
      std::istringstream is(t);
      if (is >> x) {
        int pos = int(is.tellg()); // Returns -1 at end of string?
        if (!(pos < 0 || pos == int(t.size())))
          throw GeographicErr("Extra text " + t.substr(pos) +
                              " at end of " + t);
        return x;
      }
    }
    for (std::string::iterator p = t.begin(); p != t.end(); ++p)
      *p = char(std::tolower(*p));
    switch (t[0]) {             // already checked that t isn't empty
    case 'f':
      if (t == "f" || t == "false") return false;
      break;
    case 'n':
      if (t == "n" || t == "nil" || t == "no") return false;
      break;
    case 'o':
      if (t == "off") return false;
      else if (t == "on") return true;
      break;
    case 't':
      if (t == "t" || t == "true") return true;
      break;
    case 'y':
      if (t == "y" || t == "yes") return true;
      break;
    default:
      break;
    }
    throw GeographicErr("Cannot decode " + t + " as a bool");
  }

  /**
   * Convert a Math::real object to a string.
   *
   * @param[in] x the value to be converted.
   * @param[in] p the precision used (default &minus;1).
   * @exception std::bad_alloc if memory for the string can't be allocated.
   * @return the string representation.
   *
   * If \e p &ge; 0, then the number fixed format is used with p bits of
   * precision.  With p < 0, there is no manipulation of the format.  This is
   * an overload of str<T> which deals with inf and nan.
   **********************************************************************/
  template<> inline std::string Utility::str<Math::real>(Math::real x, int p) {
    using std::isfinite;
    if (!isfinite(x))
      return x < 0 ? std::string("-inf") :
        (x > 0 ? std::string("inf") : std::string("nan"));
    std::ostringstream s;
    if (p >= 0) s << std::fixed << std::setprecision(p);
    s << x; return s.str();
  }

} // namespace GeographicLib

#if defined(_MSC_VER)
#  pragma warning (pop)
#endif

#endif  // GEOGRAPHICLIB_UTILITY_HPP
