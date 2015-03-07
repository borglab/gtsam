/**
 * \file Math.hpp
 * \brief Header for GeographicLib::Math class
 *
 * Copyright (c) Charles Karney (2008-2011) <charles@karney.com> and licensed
 * under the MIT/X11 License.  For more information, see
 * http://geographiclib.sourceforge.net/
 **********************************************************************/

// Constants.hpp includes Math.hpp.  Place this include outside Math.hpp's
// include guard to enforce this ordering.
#include <GeographicLib/Constants.hpp>

#if !defined(GEOGRAPHICLIB_MATH_HPP)
#define GEOGRAPHICLIB_MATH_HPP 1

/**
 * Are C++11 math functions available?
 **********************************************************************/
#if !defined(GEOGRAPHICLIB_CPLUSPLUS11_MATH)
#  if defined(__GNUC__) && __GNUC__ == 4 && __GNUC_MINOR__ >= 8 \
  && __cplusplus >= 201103
#    define GEOGRAPHICLIB_CPLUSPLUS11_MATH 1
#  elif defined(_MSC_VER) && _MSC_VER >= 1800
#    define GEOGRAPHICLIB_CPLUSPLUS11_MATH 1
#  else
#    define GEOGRAPHICLIB_CPLUSPLUS11_MATH 0
#  endif
#endif

#if !defined(WORDS_BIGENDIAN)
#  define WORDS_BIGENDIAN 0
#endif

#if !defined(HAVE_LONG_DOUBLE)
#  define HAVE_LONG_DOUBLE 0
#endif

#if !defined(GEOGRAPHICLIB_PRECISION)
/**
 * The precision of floating point numbers used in %GeographicLib.  1 means
 * float (single precision); 2 (the default) means double; 3 means long double;
 * 4 is reserved for quadruple precision.  Nearly all the testing has been
 * carried out with doubles and that's the recommended configuration.  In order
 * for long double to be used, HAVE_LONG_DOUBLE needs to be defined.  Note that
 * with Microsoft Visual Studio, long double is the same as double.
 **********************************************************************/
#  define GEOGRAPHICLIB_PRECISION 2
#endif

#include <cmath>
#include <algorithm>
#include <limits>
#if defined(_LIBCPP_VERSION)
#include <type_traits>
#endif

namespace GeographicLib {

  /**
   * \brief Mathematical functions needed by %GeographicLib
   *
   * Define mathematical functions in order to localize system dependencies and
   * to provide generic versions of the functions.  In addition define a real
   * type to be used by %GeographicLib.
   *
   * Example of use:
   * \include example-Math.cpp
   **********************************************************************/
  class GEOGRAPHICLIB_EXPORT Math {
  private:
    void dummy() {
      STATIC_ASSERT(GEOGRAPHICLIB_PRECISION >= 1 &&
                    GEOGRAPHICLIB_PRECISION <= 3,
                    "Bad value of precision");
    }
    Math();                     // Disable constructor
  public:

#if HAVE_LONG_DOUBLE
    /**
     * The extended precision type for real numbers, used for some testing.
     * This is long double on computers with this type; otherwise it is double.
     **********************************************************************/
    typedef long double extended;
#else
    typedef double extended;
#endif

#if GEOGRAPHICLIB_PRECISION == 2
    /**
     * The real type for %GeographicLib. Nearly all the testing has been done
     * with \e real = double.  However, the algorithms should also work with
     * float and long double (where available).  (<b>CAUTION</b>: reasonable
     * accuracy typically cannot be obtained using floats.)
     **********************************************************************/
    typedef double real;
#elif GEOGRAPHICLIB_PRECISION == 1
    typedef float real;
#elif GEOGRAPHICLIB_PRECISION == 3
    typedef extended real;
#else
    typedef double real;
#endif

    /**
     * Number of additional decimal digits of precision of real relative to
     * double (0 for float).
     **********************************************************************/
    static const int extradigits =
      std::numeric_limits<real>::digits10 >
      std::numeric_limits<double>::digits10 ?
      std::numeric_limits<real>::digits10 -
      std::numeric_limits<double>::digits10 : 0;

    /**
     * true if the machine is big-endian.
     **********************************************************************/
    static const bool bigendian = WORDS_BIGENDIAN;

    /**
     * @tparam T the type of the returned value.
     * @return &pi;.
     **********************************************************************/
    template<typename T> static inline T pi() throw()
    { return std::atan2(T(0), -T(1)); }
    /**
     * A synonym for pi<real>().
     **********************************************************************/
    static inline real pi() throw() { return pi<real>(); }

    /**
     * @tparam T the type of the returned value.
     * @return the number of radians in a degree.
     **********************************************************************/
    template<typename T> static inline T degree() throw()
    { return pi<T>() / T(180); }
    /**
     * A synonym for degree<real>().
     **********************************************************************/
    static inline real degree() throw() { return degree<real>(); }

    /**
     * Square a number.
     *
     * @tparam T the type of the argument and the returned value.
     * @param[in] x
     * @return <i>x</i><sup>2</sup>.
     **********************************************************************/
    template<typename T> static inline T sq(T x) throw()
    { return x * x; }

#if defined(DOXYGEN)
    /**
     * The hypotenuse function avoiding underflow and overflow.
     *
     * @tparam T the type of the arguments and the returned value.
     * @param[in] x
     * @param[in] y
     * @return sqrt(<i>x</i><sup>2</sup> + <i>y</i><sup>2</sup>).
     **********************************************************************/
    template<typename T> static inline T hypot(T x, T y) throw() {
      x = std::abs(x); y = std::abs(y);
      T a = (std::max)(x, y), b = (std::min)(x, y) / (a ? a : 1);
      return a * std::sqrt(1 + b * b);
      // For an alternative (square-root free) method see
      // C. Moler and D. Morrision (1983) http://dx.doi.org/10.1147/rd.276.0577
      // and A. A. Dubrulle (1983) http://dx.doi.org/10.1147/rd.276.0582
    }
#elif GEOGRAPHICLIB_CPLUSPLUS11_MATH || (defined(_MSC_VER) && _MSC_VER >= 1700)
    template<typename T> static inline T hypot(T x, T y) throw()
    { return std::hypot(x, y); }
#  if HAVE_LONG_DOUBLE && defined(_MSC_VER) && _MSC_VER == 1700
    // Visual C++ 11 doesn't have a long double overload for std::hypot --
    // reported to MS on 2013-07-18
    // http://connect.microsoft.com/VisualStudio/feedback/details/794416
    // suppress the resulting "loss of data warning" with
    static inline long double hypot(long double x, long double y) throw()
    { return std::hypot(double(x), double(y)); }
#  endif
#elif defined(_MSC_VER)
    static inline double hypot(double x, double y) throw()
    { return _hypot(x, y); }
#  if _MSC_VER < 1400
    // Visual C++ 7.1/VS .NET 2003 does not have _hypotf()
    static inline float hypot(float x, float y) throw()
    { return float(_hypot(x, y)); }
#  else
    static inline float hypot(float x, float y) throw()
    { return _hypotf(x, y); }
#  endif
#  if HAVE_LONG_DOUBLE
    static inline long double hypot(long double x, long double y) throw()
    { return _hypot(double(x), double(y)); } // Suppress loss of data warning
#  endif
#else
    // Use overloading to define generic versions
    static inline double hypot(double x, double y) throw()
    { return ::hypot(x, y); }
    static inline float hypot(float x, float y) throw()
    { return ::hypotf(x, y); }
#  if HAVE_LONG_DOUBLE
    static inline long double hypot(long double x, long double y) throw()
    { return ::hypotl(x, y); }
#  endif
#endif

#if defined(DOXYGEN) || (defined(_MSC_VER) && !GEOGRAPHICLIB_CPLUSPLUS11_MATH)
    /**
     * exp(\e x) &minus; 1 accurate near \e x = 0.  This is taken from
     * N. J. Higham, Accuracy and Stability of Numerical Algorithms, 2nd
     * Edition (SIAM, 2002), Sec 1.14.1, p 19.
     *
     * @tparam T the type of the argument and the returned value.
     * @param[in] x
     * @return exp(\e x) &minus; 1.
     **********************************************************************/
    template<typename T> static inline T expm1(T x) throw() {
      volatile T
        y = std::exp(x),
        z = y - 1;
      // The reasoning here is similar to that for log1p.  The expression
      // mathematically reduces to exp(x) - 1, and the factor z/log(y) = (y -
      // 1)/log(y) is a slowly varying quantity near y = 1 and is accurately
      // computed.
      return std::abs(x) > 1 ? z : (z == 0 ? x : x * z / std::log(y));
    }
#elif GEOGRAPHICLIB_CPLUSPLUS11_MATH
    template<typename T> static inline T expm1(T x) throw()
    { return std::expm1(x); }
#else
    static inline double expm1(double x) throw() { return ::expm1(x); }
    static inline float expm1(float x) throw() { return ::expm1f(x); }
#  if HAVE_LONG_DOUBLE
    static inline long double expm1(long double x) throw()
    { return ::expm1l(x); }
#  endif
#endif

#if defined(DOXYGEN) || (defined(_MSC_VER) && !GEOGRAPHICLIB_CPLUSPLUS11_MATH)
    /**
     * log(1 + \e x) accurate near \e x = 0.
     *
     * This is taken from D. Goldberg,
     * <a href="http://dx.doi.org/10.1145/103162.103163">What every computer
     * scientist should know about floating-point arithmetic</a> (1991),
     * Theorem 4.  See also, Higham (op. cit.), Answer to Problem 1.5, p 528.
     *
     * @tparam T the type of the argument and the returned value.
     * @param[in] x
     * @return log(1 + \e x).
     **********************************************************************/
    template<typename T> static inline T log1p(T x) throw() {
      volatile T
        y = 1 + x,
        z = y - 1;
      // Here's the explanation for this magic: y = 1 + z, exactly, and z
      // approx x, thus log(y)/z (which is nearly constant near z = 0) returns
      // a good approximation to the true log(1 + x)/x.  The multiplication x *
      // (log(y)/z) introduces little additional error.
      return z == 0 ? x : x * std::log(y) / z;
    }
#elif GEOGRAPHICLIB_CPLUSPLUS11_MATH
    template<typename T> static inline T log1p(T x) throw()
    { return std::log1p(x); }
#else
    static inline double log1p(double x) throw() { return ::log1p(x); }
    static inline float log1p(float x) throw() { return ::log1pf(x); }
#  if HAVE_LONG_DOUBLE
    static inline long double log1p(long double x) throw()
    { return ::log1pl(x); }
#  endif
#endif

#if defined(DOXYGEN) || (defined(_MSC_VER) && !GEOGRAPHICLIB_CPLUSPLUS11_MATH)
    /**
     * The inverse hyperbolic sine function.  This is defined in terms of
     * Math::log1p(\e x) in order to maintain accuracy near \e x = 0.  In
     * addition, the odd parity of the function is enforced.
     *
     * @tparam T the type of the argument and the returned value.
     * @param[in] x
     * @return asinh(\e x).
     **********************************************************************/
    template<typename T> static inline T asinh(T x) throw() {
      T y = std::abs(x);     // Enforce odd parity
      y = log1p(y * (1 + y/(hypot(T(1), y) + 1)));
      return x < 0 ? -y : y;
    }
#elif GEOGRAPHICLIB_CPLUSPLUS11_MATH
    template<typename T> static inline T asinh(T x) throw()
    { return std::asinh(x); }
#else
    static inline double asinh(double x) throw() { return ::asinh(x); }
    static inline float asinh(float x) throw() { return ::asinhf(x); }
#  if HAVE_LONG_DOUBLE
    static inline long double asinh(long double x) throw()
    { return ::asinhl(x); }
#  endif
#endif

#if defined(DOXYGEN) || (defined(_MSC_VER) && !GEOGRAPHICLIB_CPLUSPLUS11_MATH)
    /**
     * The inverse hyperbolic tangent function.  This is defined in terms of
     * Math::log1p(\e x) in order to maintain accuracy near \e x = 0.  In
     * addition, the odd parity of the function is enforced.
     *
     * @tparam T the type of the argument and the returned value.
     * @param[in] x
     * @return atanh(\e x).
     **********************************************************************/
    template<typename T> static inline T atanh(T x) throw() {
      T y = std::abs(x);     // Enforce odd parity
      y = log1p(2 * y/(1 - y))/2;
      return x < 0 ? -y : y;
    }
#elif GEOGRAPHICLIB_CPLUSPLUS11_MATH
    template<typename T> static inline T atanh(T x) throw()
    { return std::atanh(x); }
#else
    static inline double atanh(double x) throw() { return ::atanh(x); }
    static inline float atanh(float x) throw() { return ::atanhf(x); }
#  if HAVE_LONG_DOUBLE
    static inline long double atanh(long double x) throw()
    { return ::atanhl(x); }
#  endif
#endif

#if defined(DOXYGEN) || (defined(_MSC_VER) && !GEOGRAPHICLIB_CPLUSPLUS11_MATH)
    /**
     * The cube root function.
     *
     * @tparam T the type of the argument and the returned value.
     * @param[in] x
     * @return the real cube root of \e x.
     **********************************************************************/
    template<typename T> static inline T cbrt(T x) throw() {
      T y = std::pow(std::abs(x), 1/T(3)); // Return the real cube root
      return x < 0 ? -y : y;
    }
#elif GEOGRAPHICLIB_CPLUSPLUS11_MATH
    template<typename T> static inline T cbrt(T x) throw()
    { return std::cbrt(x); }
#else
    static inline double cbrt(double x) throw() { return ::cbrt(x); }
    static inline float cbrt(float x) throw() { return ::cbrtf(x); }
#  if HAVE_LONG_DOUBLE
    static inline long double cbrt(long double x) throw() { return ::cbrtl(x); }
#  endif
#endif

    /**
     * The error-free sum of two numbers.
     *
     * @tparam T the type of the argument and the returned value.
     * @param[in] u
     * @param[in] v
     * @param[out] t the exact error given by (\e u + \e v) - \e s.
     * @return \e s = round(\e u + \e v).
     *
     * See D. E. Knuth, TAOCP, Vol 2, 4.2.2, Theorem B.  (Note that \e t can be
     * the same as one of the first two arguments.)
     **********************************************************************/
    template<typename T> static inline T sum(T u, T v, T& t) throw() {
      volatile T s = u + v;
      volatile T up = s - v;
      volatile T vpp = s - up;
      up -= u;
      vpp -= v;
      t = -(up + vpp);
      // u + v =       s      + t
      //       = round(u + v) + t
      return s;
    }

    /**
     * Normalize an angle (restricted input range).
     *
     * @tparam T the type of the argument and returned value.
     * @param[in] x the angle in degrees.
     * @return the angle reduced to the range [&minus;180&deg;, 180&deg;).
     *
     * \e x must lie in [&minus;540&deg;, 540&deg;).
     **********************************************************************/
    template<typename T> static inline T AngNormalize(T x) throw()
    { return x >= 180 ? x - 360 : (x < -180 ? x + 360 : x); }

    /**
     * Normalize an arbitrary angle.
     *
     * @tparam T the type of the argument and returned value.
     * @param[in] x the angle in degrees.
     * @return the angle reduced to the range [&minus;180&deg;, 180&deg;).
     *
     * The range of \e x is unrestricted.
     **********************************************************************/
    template<typename T> static inline T AngNormalize2(T x) throw()
    { return AngNormalize<T>(std::fmod(x, T(360))); }

    /**
     * Difference of two angles reduced to [&minus;180&deg;, 180&deg;]
     *
     * @tparam T the type of the arguments and returned value.
     * @param[in] x the first angle in degrees.
     * @param[in] y the second angle in degrees.
     * @return \e y &minus; \e x, reduced to the range [&minus;180&deg;,
     *   180&deg;].
     *
     * \e x and \e y must both lie in [&minus;180&deg;, 180&deg;].  The result
     * is equivalent to computing the difference exactly, reducing it to
     * (&minus;180&deg;, 180&deg;] and rounding the result.  Note that this
     * prescription allows &minus;180&deg; to be returned (e.g., if \e x is
     * tiny and negative and \e y = 180&deg;).
     **********************************************************************/
    template<typename T> static inline T AngDiff(T x, T y) throw() {
      T t, d = sum(-x, y, t);
      if ((d - T(180)) + t > T(0)) // y - x > 180
        d -= T(360);            // exact
      else if ((d + T(180)) + t <= T(0)) // y - x <= -180
        d += T(360);            // exact
      return d + t;
    }

#if defined(DOXYGEN)
    /**
     * Test for finiteness.
     *
     * @tparam T the type of the argument.
     * @param[in] x
     * @return true if number is finite, false if NaN or infinite.
     **********************************************************************/
    template<typename T> static inline bool isfinite(T x) throw() {
      return std::abs(x) <= (std::numeric_limits<T>::max)();
    }
#elif (defined(_MSC_VER) && !GEOGRAPHICLIB_CPLUSPLUS11_MATH)
    template<typename T> static inline bool isfinite(T x) throw() {
      return _finite(double(x)) != 0;
    }
#elif defined(_LIBCPP_VERSION)
    // libc++ implements std::isfinite() as a template that only allows
    // floating-point types.  isfinite is invoked by Utility::str to format
    // numbers conveniently and this allows integer arguments, so we need to
    // allow Math::isfinite to work on integers.
    template<typename T> static inline
    typename std::enable_if<std::is_floating_point<T>::value, bool>::type
      isfinite(T x) throw() {
      return std::isfinite(x);
    }
    template<typename T> static inline
    typename std::enable_if<!std::is_floating_point<T>::value, bool>::type
      isfinite(T /*x*/) throw() {
      return true;
    }
#else
    template<typename T> static inline bool isfinite(T x) throw() {
      return std::isfinite(x);
    }
#endif

    /**
     * The NaN (not a number)
     *
     * @tparam T the type of the returned value.
     * @return NaN if available, otherwise return the max real of type T.
     **********************************************************************/
    template<typename T> static inline T NaN() throw() {
      return std::numeric_limits<T>::has_quiet_NaN ?
        std::numeric_limits<T>::quiet_NaN() :
        (std::numeric_limits<T>::max)();
    }
    /**
     * A synonym for NaN<real>().
     **********************************************************************/
    static inline real NaN() throw() { return NaN<real>(); }

    /**
     * Test for NaN.
     *
     * @tparam T the type of the argument.
     * @param[in] x
     * @return true if argument is a NaN.
     **********************************************************************/
    template<typename T> static inline bool isnan(T x) throw() {
#if defined(DOXYGEN) || (defined(_MSC_VER) && !GEOGRAPHICLIB_CPLUSPLUS11_MATH)
      return x != x;
#else
      return std::isnan(x);
#endif
    }

    /**
     * Infinity
     *
     * @tparam T the type of the returned value.
     * @return infinity if available, otherwise return the max real.
     **********************************************************************/
    template<typename T> static inline T infinity() throw() {
      return std::numeric_limits<T>::has_infinity ?
        std::numeric_limits<T>::infinity() :
        (std::numeric_limits<T>::max)();
    }
    /**
     * A synonym for infinity<real>().
     **********************************************************************/
    static inline real infinity() throw() { return infinity<real>(); }

    /**
     * Swap the bytes of a quantity
     *
     * @tparam T the type of the argument and the returned value.
     * @param[in] x
     * @return x with its bytes swapped.
     **********************************************************************/
    template<typename T> static inline T swab(T x) {
      union {
        T r;
        unsigned char c[sizeof(T)];
      } b;
      b.r = x;
      for (int i = sizeof(T)/2; i--; )
        std::swap(b.c[i], b.c[sizeof(T) - 1 - i]);
      return b.r;
    }
  };

} // namespace GeographicLib

#endif  // GEOGRAPHICLIB_MATH_HPP
