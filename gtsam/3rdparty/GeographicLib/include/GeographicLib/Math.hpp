/**
 * \file Math.hpp
 * \brief Header for GeographicLib::Math class
 *
 * Copyright (c) Charles Karney (2008-2024) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

// Constants.hpp includes Math.hpp.  Place this include outside Math.hpp's
// include guard to enforce this ordering.
#include <GeographicLib/Constants.hpp>

#if !defined(GEOGRAPHICLIB_MATH_HPP)
#define GEOGRAPHICLIB_MATH_HPP 1

#if !defined(GEOGRAPHICLIB_WORDS_BIGENDIAN)
#  define GEOGRAPHICLIB_WORDS_BIGENDIAN 0
#endif

#if !defined(GEOGRAPHICLIB_HAVE_LONG_DOUBLE)
#  define GEOGRAPHICLIB_HAVE_LONG_DOUBLE 0
#endif

#if !defined(GEOGRAPHICLIB_PRECISION)
/**
 * The precision of floating point numbers used in %GeographicLib.  1 means
 * float (single precision); 2 (the default) means double; 3 means long double;
 * 4 is reserved for quadruple precision.  Nearly all the testing has been
 * carried out with doubles and that's the recommended configuration.  In order
 * for long double to be used, GEOGRAPHICLIB_HAVE_LONG_DOUBLE needs to be
 * defined.  Note that with Microsoft Visual Studio, long double is the same as
 * double.
 **********************************************************************/
#  define GEOGRAPHICLIB_PRECISION 2
#endif

#include <cmath>
#include <algorithm>
#include <limits>

#if GEOGRAPHICLIB_PRECISION == 4
#include <memory>
#include <boost/version.hpp>
#include <boost/multiprecision/float128.hpp>
#include <boost/math/special_functions.hpp>
#elif GEOGRAPHICLIB_PRECISION == 5
#include <mpreal.h>
#endif

#if GEOGRAPHICLIB_PRECISION > 3
// volatile keyword makes no sense for multiprec types
#define GEOGRAPHICLIB_VOLATILE
// Signal a convergence failure with multiprec types by throwing an exception
// at loop exit.
#define GEOGRAPHICLIB_PANIC(msg) \
  (throw GeographicLib::GeographicErr(msg), false)
#else
#define GEOGRAPHICLIB_VOLATILE volatile
// Ignore convergence failures with standard floating points types by allowing
// loop to exit cleanly.
#define GEOGRAPHICLIB_PANIC(msg) false
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
    void dummy();               // Static check for GEOGRAPHICLIB_PRECISION
    Math() = delete;            // Disable constructor
  public:

#if GEOGRAPHICLIB_HAVE_LONG_DOUBLE
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
#elif GEOGRAPHICLIB_PRECISION == 4
    typedef boost::multiprecision::float128 real;
#elif GEOGRAPHICLIB_PRECISION == 5
    typedef mpfr::mpreal real;
#else
    typedef double real;
#endif

    /**
     * The constants defining the standard (Babylonian) meanings of degrees,
     * minutes, and seconds, for angles.  Read the constants as follows (for
     * example): \e ms = 60 is the ratio 1 minute / 1 second.  The
     * abbreviations are
     * - \e t a whole turn (360&deg;)
     * - \e h a half turn (180&deg;)
     * - \e q a quarter turn (a right angle = 90&deg;)
     * - \e d a degree
     * - \e m a minute
     * - \e s a second
     * .
     * Note that degree() is ratio 1 degree / 1 radian, thus, for example,
     * Math::degree() * Math::qd is the ratio 1 quarter turn / 1 radian =
     * &pi;/2.
     *
     * Defining all these in one place would mean that it's simple to convert
     * to the centesimal system for measuring angles.  The DMS class assumes
     * that Math::dm and Math::ms are less than or equal to 100 (so that two
     * digits suffice for the integer parts of the minutes and degrees
     * components of an angle).  Switching to the centesimal convention will
     * break most of the tests.  Also the normal definition of degree is baked
     * into some classes, e.g., UTMUPS, MGRS, Georef, Geohash, etc.
     **********************************************************************/
#if __cplusplus >= 201703L
    static inline constexpr int qd = 90;      ///< degrees per quarter turn
    static inline constexpr int dm = 60;      ///< minutes per degree
    static inline constexpr int ms = 60;      ///< seconds per minute
    static inline constexpr int hd = 2 * qd;  ///< degrees per half turn
    static inline constexpr int td = 2 * hd;  ///< degrees per turn
    static inline constexpr int ds = dm * ms; ///< seconds per degree
#elif GEOGRAPHICLIB_PRECISION < 4
    static constexpr int qd = 90;      ///< degrees per quarter turn
    static constexpr int dm = 60;      ///< minutes per degree
    static constexpr int ms = 60;      ///< seconds per minute
    static constexpr int hd = 2 * qd;  ///< degrees per half turn
    static constexpr int td = 2 * hd;  ///< degrees per turn
    static constexpr int ds = dm * ms; ///< seconds per degree
#else
    enum dms {
      qd = 90,                  ///< degrees per quarter turn
      dm = 60,                  ///< minutes per degree
      ms = 60,                  ///< seconds per minute
      hd = 2 * qd,              ///< degrees per half turn
      td = 2 * hd,              ///< degrees per turn
      ds = dm * ms              ///< seconds per degree
    };
#endif

    /**
     * @return the number of bits of precision in a real number.
     **********************************************************************/
    static int digits();

    /**
     * Set the binary precision of a real number.
     *
     * @param[in] ndigits the number of bits of precision.
     * @return the resulting number of bits of precision.
     *
     * This only has an effect when GEOGRAPHICLIB_PRECISION = 5.  See also
     * Utility::set_digits for caveats about when this routine should be
     * called.
     **********************************************************************/
    static int set_digits(int ndigits);

    /**
     * @return the number of decimal digits of precision in a real number.
     **********************************************************************/
    static int digits10();

    /**
     * Number of additional decimal digits of precision for real relative to
     * double (0 for float).
     **********************************************************************/
    static int extra_digits();

    /**
     * true if the machine is big-endian.
     **********************************************************************/
    static const bool bigendian = GEOGRAPHICLIB_WORDS_BIGENDIAN;

    /**
     * @tparam T the type of the returned value.
     * @return &pi;.
     **********************************************************************/
    template<typename T = real> static T pi() {
      using std::atan2;
      static const T pi = atan2(T(0), T(-1));
      return pi;
    }

    /**
     * @tparam T the type of the returned value.
     * @return the number of radians in a degree.
     **********************************************************************/
    template<typename T = real> static T degree() {
      static const T degree = pi<T>() / T(hd);
      return degree;
    }

    /**
     * Square a number.
     *
     * @tparam T the type of the argument and the returned value.
     * @param[in] x
     * @return <i>x</i><sup>2</sup>.
     **********************************************************************/
    template<typename T> static T sq(T x)
    { return x * x; }

    /**
     * Normalize a two-vector.
     *
     * @tparam T the type of the argument and the returned value.
     * @param[in,out] x on output set to <i>x</i>/hypot(<i>x</i>, <i>y</i>).
     * @param[in,out] y on output set to <i>y</i>/hypot(<i>x</i>, <i>y</i>).
     **********************************************************************/
    template<typename T> static void norm(T& x, T& y) {
#if defined(_MSC_VER) && defined(_M_IX86)
      // hypot for Visual Studio (A=win32) fails monotonicity, e.g., with
      //   x  = 0.6102683302836215
      //   y1 = 0.7906090004346522
      //   y2 = y1 + 1e-16
      // the test
      //   hypot(x, y2) >= hypot(x, y1)
      // fails.  Reported 2021-03-14:
      //   https://developercommunity.visualstudio.com/t/1369259
      // See also:
      //   https://bugs.python.org/issue43088
      using std::sqrt; T h = sqrt(x * x + y * y);
#else
      using std::hypot; T h = hypot(x, y);
#endif
      x /= h; y /= h;
    }

    /**
     * The error-free sum of two numbers.
     *
     * @tparam T the type of the argument and the returned value.
     * @param[in] u
     * @param[in] v
     * @param[out] t the exact error given by (\e u + \e v) - \e s.
     * @return \e s = round(\e u + \e v).
     *
     * See D. E. Knuth, TAOCP, Vol 2, 4.2.2, Theorem B.
     *
     * \note \e t can be the same as one of the first two arguments.
     **********************************************************************/
    template<typename T> static T sum(T u, T v, T& t);

    /**
     * Evaluate a polynomial.
     *
     * @tparam T the type of the arguments and returned value.
     * @param[in] N the order of the polynomial.
     * @param[in] p the coefficient array (of size \e N + 1) with
     *   <i>p</i><sub>0</sub> being coefficient of <i>x</i><sup><i>N</i></sup>.
     * @param[in] x the variable.
     * @return the value of the polynomial.
     *
     * Evaluate &sum;<sub><i>n</i>=0..<i>N</i></sub>
     * <i>p</i><sub><i>n</i></sub> <i>x</i><sup><i>N</i>&minus;<i>n</i></sup>.
     * Return 0 if \e N &lt; 0.  Return <i>p</i><sub>0</sub>, if \e N = 0 (even
     * if \e x is infinite or a nan).  The evaluation uses Horner's method.
     **********************************************************************/
    template<typename T> static T polyval(int N, const T p[], T x) {
      // This used to employ Math::fma; but that's too slow and it seemed not
      // to improve the accuracy noticeably.  This might change when there's
      // direct hardware support for fma.
      T y = N < 0 ? 0 : *p++;
      while (--N >= 0) y = y * x + *p++;
      return y;
    }

    /**
     * Normalize an angle.
     *
     * @tparam T the type of the argument and returned value.
     * @param[in] x the angle in degrees.
     * @return the angle reduced to the range [&minus;180&deg;, 180&deg;].
     *
     * The range of \e x is unrestricted.  If the result is &plusmn;0&deg; or
     * &plusmn;180&deg; then the sign is the sign of \e x.
     **********************************************************************/
    template<typename T> static T AngNormalize(T x);

    /**
     * Normalize a latitude.
     *
     * @tparam T the type of the argument and returned value.
     * @param[in] x the angle in degrees.
     * @return x if it is in the range [&minus;90&deg;, 90&deg;], otherwise
     *   return NaN.
     **********************************************************************/
    template<typename T> static T LatFix(T x)
    { using std::fabs; return fabs(x) > T(qd) ? NaN<T>() : x; }

    /**
     * The exact difference of two angles reduced to
     * [&minus;180&deg;, 180&deg;].
     *
     * @tparam T the type of the arguments and returned value.
     * @param[in] x the first angle in degrees.
     * @param[in] y the second angle in degrees.
     * @param[out] e the error term in degrees.
     * @return \e d, the truncated value of \e y &minus; \e x.
     *
     * This computes \e z = \e y &minus; \e x exactly, reduced to
     * [&minus;180&deg;, 180&deg;]; and then sets \e z = \e d + \e e where \e d
     * is the nearest representable number to \e z and \e e is the truncation
     * error.  If \e z = &plusmn;0&deg; or &plusmn;180&deg;, then the sign of
     * \e d is given by the sign of \e y &minus; \e x.  The maximum absolute
     * value of \e e is 2<sup>&minus;26</sup> (for doubles).
     **********************************************************************/
    template<typename T> static T AngDiff(T x, T y, T& e);

    /**
     * Difference of two angles reduced to [&minus;180&deg;, 180&deg;]
     *
     * @tparam T the type of the arguments and returned value.
     * @param[in] x the first angle in degrees.
     * @param[in] y the second angle in degrees.
     * @return \e y &minus; \e x, reduced to the range [&minus;180&deg;,
     *   180&deg;].
     *
     * The result is equivalent to computing the difference exactly, reducing
     * it to [&minus;180&deg;, 180&deg;] and rounding the result.
     **********************************************************************/
    template<typename T> static T AngDiff(T x, T y)
    { T e; return AngDiff(x, y, e); }

    /**
     * Coarsen a value close to zero.
     *
     * @tparam T the type of the argument and returned value.
     * @param[in] x
     * @return the coarsened value.
     *
     * The makes the smallest gap in \e x = 1/16 &minus; nextafter(1/16, 0) =
     * 1/2<sup>57</sup> for doubles = 0.8 pm on the earth if \e x is an angle
     * in degrees.  (This is about 2000 times more resolution than we get with
     * angles around 90&deg;.)  We use this to avoid having to deal with near
     * singular cases when \e x is non-zero but tiny (e.g.,
     * 10<sup>&minus;200</sup>).  This sign of &plusmn;0 is preserved.
     **********************************************************************/
    template<typename T> static T AngRound(T x);

    /**
     * Evaluate the sine and cosine function with the argument in degrees
     *
     * @tparam T the type of the arguments.
     * @param[in] x in degrees.
     * @param[out] sinx sin(<i>x</i>).
     * @param[out] cosx cos(<i>x</i>).
     *
     * The results obey exactly the elementary properties of the trigonometric
     * functions, e.g., sin 9&deg; = cos 81&deg; = &minus; sin 123456789&deg;.
     * If x = &minus;0 or a negative multiple of 180&deg;, then \e sinx =
     * &minus;0; this is the only case where &minus;0 is returned.
     **********************************************************************/
    template<typename T> static void sincosd(T x, T& sinx, T& cosx);

    /**
     * Evaluate the sine and cosine with reduced argument plus correction
     *
     * @tparam T the type of the arguments.
     * @param[in] x reduced angle in degrees.
     * @param[in] t correction in degrees.
     * @param[out] sinx sin(<i>x</i> + <i>t</i>).
     * @param[out] cosx cos(<i>x</i> + <i>t</i>).
     *
     * This is a variant of Math::sincosd allowing a correction to the angle to
     * be supplied.  \e x must be in [&minus;180&deg;, 180&deg;] and \e t is
     * assumed to be a <i>small</i> correction.  Math::AngRound is applied to
     * the reduced angle to prevent problems with \e x + \e t being extremely
     * close but not exactly equal to one of the four cardinal directions.
     **********************************************************************/
    template<typename T> static void sincosde(T x, T t, T& sinx, T& cosx);

    /**
     * Evaluate the sine function with the argument in degrees
     *
     * @tparam T the type of the argument and the returned value.
     * @param[in] x in degrees.
     * @return sin(<i>x</i>).
     *
     * The result is +0 for \e x = +0 and positive multiples of 180&deg;.  The
     * result is &minus;0 for \e x = -0 and negative multiples of 180&deg;.
     **********************************************************************/
    template<typename T> static T sind(T x);

    /**
     * Evaluate the cosine function with the argument in degrees
     *
     * @tparam T the type of the argument and the returned value.
     * @param[in] x in degrees.
     * @return cos(<i>x</i>).
     *
     * The result is +0 for \e x an odd multiple of 90&deg;.
     **********************************************************************/
    template<typename T> static T cosd(T x);

    /**
     * Evaluate the tangent function with the argument in degrees
     *
     * @tparam T the type of the argument and the returned value.
     * @param[in] x in degrees.
     * @return tan(<i>x</i>).
     *
     * If \e x is an odd multiple of 90&deg;, then a suitably large (but
     * finite) value is returned.
     **********************************************************************/
    template<typename T> static T tand(T x);

    /**
     * Evaluate the atan2 function with the result in degrees
     *
     * @tparam T the type of the arguments and the returned value.
     * @param[in] y
     * @param[in] x
     * @return atan2(<i>y</i>, <i>x</i>) in degrees.
     *
     * The result is in the range [&minus;180&deg; 180&deg;].  N.B.,
     * atan2d(&plusmn;0, &minus;1) = &plusmn;180&deg;.
     **********************************************************************/
    template<typename T> static T atan2d(T y, T x);

    /**
     * Evaluate the atan function with the result in degrees
     *
     * @tparam T the type of the argument and the returned value.
     * @param[in] x
     * @return atan(<i>x</i>) in degrees.
     **********************************************************************/
    template<typename T> static T atand(T x);

    /**
     * Evaluate <i>e</i> atanh(<i>e x</i>)
     *
     * @tparam T the type of the argument and the returned value.
     * @param[in] x
     * @param[in] es the signed eccentricity =  sign(<i>e</i><sup>2</sup>)
     *    sqrt(|<i>e</i><sup>2</sup>|)
     * @return <i>e</i> atanh(<i>e x</i>)
     *
     * If <i>e</i><sup>2</sup> is negative (<i>e</i> is imaginary), the
     * expression is evaluated in terms of atan.
     **********************************************************************/
    template<typename T> static T eatanhe(T x, T es);

    /**
     * tan&chi; in terms of tan&phi;
     *
     * @tparam T the type of the argument and the returned value.
     * @param[in] tau &tau; = tan&phi;
     * @param[in] es the signed eccentricity = sign(<i>e</i><sup>2</sup>)
     *   sqrt(|<i>e</i><sup>2</sup>|)
     * @return &tau;&prime; = tan&chi;
     *
     * See Eqs. (7--9) of
     * C. F. F. Karney,
     * <a href="https://doi.org/10.1007/s00190-011-0445-3">
     * Transverse Mercator with an accuracy of a few nanometers,</a>
     * J. Geodesy 85(8), 475--485 (Aug. 2011)
     * (preprint
     * <a href="https://arxiv.org/abs/1002.1417">arXiv:1002.1417</a>).
     **********************************************************************/
    template<typename T> static T taupf(T tau, T es);

    /**
     * tan&phi; in terms of tan&chi;
     *
     * @tparam T the type of the argument and the returned value.
     * @param[in] taup &tau;&prime; = tan&chi;
     * @param[in] es the signed eccentricity = sign(<i>e</i><sup>2</sup>)
     *   sqrt(|<i>e</i><sup>2</sup>|)
     * @return &tau; = tan&phi;
     *
     * See Eqs. (19--21) of
     * C. F. F. Karney,
     * <a href="https://doi.org/10.1007/s00190-011-0445-3">
     * Transverse Mercator with an accuracy of a few nanometers,</a>
     * J. Geodesy 85(8), 475--485 (Aug. 2011)
     * (preprint
     * <a href="https://arxiv.org/abs/1002.1417">arXiv:1002.1417</a>).
     **********************************************************************/
    template<typename T> static T tauf(T taup, T es);

    /**
     * Implement hypot with 3 parameters
     *
     * @tparam T the type of the argument and the returned value.
     * @param[in] x
     * @param[in] y
     * @param[in] z
     * @return sqrt(<i>x</i><sup>2</sup> + <i>y</i><sup>2</sup> +
     *   <i>z</i><sup>2</sup>).
     **********************************************************************/
    template<typename T> static T hypot3(T x, T y, T z);

    /**
     * The NaN (not a number)
     *
     * @tparam T the type of the returned value.
     * @return NaN if available, otherwise return the max real of type T.
     **********************************************************************/
    template<typename T = real> static T NaN();

    /**
     * Infinity
     *
     * @tparam T the type of the returned value.
     * @return infinity if available, otherwise return the max real.
     **********************************************************************/
    template<typename T = real> static T infinity();

    /**
     * Swap the bytes of a quantity
     *
     * @tparam T the type of the argument and the returned value.
     * @param[in] x
     * @return x with its bytes swapped.
     **********************************************************************/
    template<typename T> static T swab(T x) {
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
