/**
 * \file Math.cpp
 * \brief Implementation for GeographicLib::Math class
 *
 * Copyright (c) Charles Karney (2015-2024) <karney@alum.mit.edu> and licensed
 * under the MIT/X11 License.  For more information, see
 * https://geographiclib.sourceforge.io/
 **********************************************************************/

#include <GeographicLib/Math.hpp>

namespace GeographicLib {

  using namespace std;

  void Math::dummy() {
    static_assert(GEOGRAPHICLIB_PRECISION >= 1 && GEOGRAPHICLIB_PRECISION <= 5,
                  "Bad value of precision");
  }

  int Math::digits() {
#if GEOGRAPHICLIB_PRECISION != 5
    return numeric_limits<real>::digits;
#else
    return numeric_limits<real>::digits();
#endif
  }

  int Math::set_digits(int ndigits) {
#if GEOGRAPHICLIB_PRECISION != 5
    (void)ndigits;
#else
    mpfr::mpreal::set_default_prec(ndigits >= 2 ? ndigits : 2);
#endif
    return digits();
  }

  int Math::digits10() {
#if GEOGRAPHICLIB_PRECISION != 5
    return numeric_limits<real>::digits10;
#else
    return numeric_limits<real>::digits10();
#endif
  }

  int Math::extra_digits() {
    return
      digits10() > numeric_limits<double>::digits10 ?
      digits10() - numeric_limits<double>::digits10 : 0;
  }

  template<typename T> T Math::sum(T u, T v, T& t) {
    GEOGRAPHICLIB_VOLATILE T s = u + v;
    GEOGRAPHICLIB_VOLATILE T up = s - v;
    GEOGRAPHICLIB_VOLATILE T vpp = s - up;
    up -= u;
    vpp -= v;
    // if s = 0, then t = 0 and give t the same sign as s
    // mpreal needs T(0) here
    t = s != 0 ? T(0) - (up + vpp) : s;
    // u + v =       s      + t
    //       = round(u + v) + t
    return s;
  }

  template<typename T> T Math::AngNormalize(T x) {
    T y = remainder(x, T(td));
#if GEOGRAPHICLIB_PRECISION == 4
    // boost-quadmath doesn't set the sign of 0 correctly, see
    // https://github.com/boostorg/multiprecision/issues/426
    // Fixed by https://github.com/boostorg/multiprecision/pull/428
    if (y == 0) y = copysign(y, x);
#endif
    return fabs(y) == T(hd) ? copysign(T(hd), x) : y;
  }

  template<typename T> T Math::AngDiff(T x, T y, T& e) {
    // Use remainder instead of AngNormalize, since we treat boundary cases
    // later taking account of the error
    T d = sum(remainder(-x, T(td)), remainder( y, T(td)), e);
    // This second sum can only change d if abs(d) < 128, so don't need to
    // apply remainder yet again.
    d = sum(remainder(d, T(td)), e, e);
    // Fix the sign if d = -180, 0, 180.
    if (d == 0 || fabs(d) == hd)
      // If e == 0, take sign from y - x
      // else (e != 0, implies d = +/-180), d and e must have opposite signs
      d = copysign(d, e == 0 ? y - x : -e);
    return d;
  }

  template<typename T> T Math::AngRound(T x) {
    static const T z = T(1)/T(16);
    GEOGRAPHICLIB_VOLATILE T y = fabs(x);
    GEOGRAPHICLIB_VOLATILE T w = z - y;
    // The compiler mustn't "simplify" z - (z - y) to y
    y = w > 0 ? z - w : y;
    return copysign(y, x);
  }

  template<typename T> void Math::sincosd(T x, T& sinx, T& cosx) {
    // In order to minimize round-off errors, this function exactly reduces
    // the argument to the range [-45, 45] before converting it to radians.
    T d, r; int q = 0;
    d = remquo(x, T(qd), &q);   // now abs(r) <= 45
    r = d * degree<T>();
    // g++ -O turns these two function calls into a call to sincos
    T s = sin(r), c = cos(r);
    if (2 * fabs(d) == qd) {
      c = sqrt(1/T(2));
      s = copysign(c, r);
    } else if (3 * fabs(d) == qd) {
      c = sqrt(T(3))/2;
      s = copysign(1/T(2), r);
    }
    switch (unsigned(q) & 3U) {
    case 0U: sinx =  s; cosx =  c; break;
    case 1U: sinx =  c; cosx = -s; break;
    case 2U: sinx = -s; cosx = -c; break;
    default: sinx = -c; cosx =  s; break; // case 3U
    }
    // http://www.open-std.org/jtc1/sc22/wg14/www/docs/n1950.pdf
    // mpreal needs T(0) here
    cosx += T(0);                            // special values from F.10.1.12
    if (sinx == 0) sinx = copysign(sinx, x); // special values from F.10.1.13
  }

  template<typename T> void Math::sincosde(T x, T t, T& sinx, T& cosx) {
    // In order to minimize round-off errors, this function exactly reduces
    // the argument to the range [-45, 45] before converting it to radians.
    // This implementation allows x outside [-180, 180], but implementations in
    // other languages may not.
    int q = 0;
    T d = AngRound(remquo(x, T(qd), &q) + t), // now abs(r) <= 45
      r = d * degree<T>();
    // g++ -O turns these two function calls into a call to sincos
    T s = sin(r), c = cos(r);
    if (2 * fabs(d) == qd) {
      c = sqrt(1/T(2));
      s = copysign(c, r);
    } else if (3 * fabs(d) == qd) {
      c = sqrt(T(3))/2;
      s = copysign(1/T(2), r);
    }
    switch (unsigned(q) & 3U) {
    case 0U: sinx =  s; cosx =  c; break;
    case 1U: sinx =  c; cosx = -s; break;
    case 2U: sinx = -s; cosx = -c; break;
    default: sinx = -c; cosx =  s; break; // case 3U
    }
    // http://www.open-std.org/jtc1/sc22/wg14/www/docs/n1950.pdf
    // mpreal needs T(0) here
    cosx += T(0);                            // special values from F.10.1.12
    if (sinx == 0) sinx = copysign(sinx, x+t); // special values from F.10.1.13
  }

  template<typename T> T Math::sind(T x) {
    // See sincosd
    int q = 0;
    T d = remquo(x, T(qd), &q), // now abs(r) <= 45
      r = d * degree<T>();
    unsigned p = unsigned(q);
    // r = p & 1U ? cos(r) : sin(r); replaced by ...
    r = p & 1U ? (2 * fabs(d) == qd ? sqrt(1/T(2)) :
                  (3 * fabs(d) == qd ? sqrt(T(3))/2 : cos(r))) :
      copysign(2 * fabs(d) == qd ? sqrt(1/T(2)) :
               (3 * fabs(d) == qd ? 1/T(2) : sin(r)), r);
    if (p & 2U) r = -r;
    if (r == 0) r = copysign(r, x);
    return r;
  }

  template<typename T> T Math::cosd(T x) {
    // See sincosd
    int q = 0;
    T d = remquo(x, T(qd), &q), // now abs(r) <= 45
      r = d * degree<T>();
    unsigned p = unsigned(q + 1);
    r = p & 1U ? (2 * fabs(d) == qd ? sqrt(1/T(2)) :
                  (3 * fabs(d) == qd ? sqrt(T(3))/2 : cos(r))) :
      copysign(2 * fabs(d) == qd ? sqrt(1/T(2)) :
               (3 * fabs(d) == qd ? 1/T(2) : sin(r)), r);
    if (p & 2U) r = -r;
    // mpreal needs T(0) here
    return T(0) + r;
  }

  template<typename T> T Math::tand(T x) {
    static const T overflow = 1 / sq(numeric_limits<T>::epsilon());
    T s, c;
    sincosd(x, s, c);
    // http://www.open-std.org/jtc1/sc22/wg14/www/docs/n1950.pdf
    T r = s / c;  // special values from F.10.1.14
    // With C++17 this becomes clamp(s / c, -overflow, overflow);
    // Use max/min here (instead of fmax/fmin) to preserve NaN
    return min(max(r, -overflow), overflow);
  }

  template<typename T> T Math::atan2d(T y, T x) {
    // In order to minimize round-off errors, this function rearranges the
    // arguments so that result of atan2 is in the range [-pi/4, pi/4] before
    // converting it to degrees and mapping the result to the correct
    // quadrant.
    int q = 0;
    if (fabs(y) > fabs(x)) { swap(x, y); q = 2; }
    if (signbit(x)) { x = -x; ++q; }
    // here x >= 0 and x >= abs(y), so angle is in [-pi/4, pi/4]
    T ang = atan2(y, x) / degree<T>();
    switch (q) {
    case 1: ang = copysign(T(hd), y) - ang; break;
    case 2: ang =            qd      - ang; break;
    case 3: ang =           -qd      + ang; break;
    default: break;
    }
    return ang;
  }

  template<typename T> T Math::atand(T x)
  { return atan2d(x, T(1)); }

  template<typename T> T Math::eatanhe(T x, T es)  {
    return es > 0 ? es * atanh(es * x) : -es * atan(es * x);
  }

  template<typename T> T Math::taupf(T tau, T es) {
    // Need this test, otherwise tau = +/-inf gives taup = nan.
    if (isfinite(tau)) {
      T tau1 = hypot(T(1), tau),
        sig = sinh( eatanhe(tau / tau1, es ) );
      return hypot(T(1), sig) * tau - sig * tau1;
    } else
      return tau;
  }

  template<typename T> T Math::tauf(T taup, T es) {
    static const int numit = 5;
    // min iterations = 1, max iterations = 2; mean = 1.95
    static const T tol = sqrt(numeric_limits<T>::epsilon()) / 10;
    static const T taumax = 2 / sqrt(numeric_limits<T>::epsilon());
    T e2m = 1 - sq(es),
      // To lowest order in e^2, taup = (1 - e^2) * tau = _e2m * tau; so use
      // tau = taup/e2m as a starting guess. Only 1 iteration is needed for
      // |lat| < 3.35 deg, otherwise 2 iterations are needed.  If, instead, tau
      // = taup is used the mean number of iterations increases to 1.999 (2
      // iterations are needed except near tau = 0).
      //
      // For large tau, taup = exp(-es*atanh(es)) * tau.  Use this as for the
      // initial guess for |taup| > 70 (approx |phi| > 89deg).  Then for
      // sufficiently large tau (such that sqrt(1+tau^2) = |tau|), we can exit
      // with the intial guess and avoid overflow problems.  This also reduces
      // the mean number of iterations slightly from 1.963 to 1.954.
      tau = fabs(taup) > 70 ? taup * exp(eatanhe(T(1), es)) : taup/e2m,
      stol = tol * fmax(T(1), fabs(taup));
    if (!(fabs(tau) < taumax)) return tau; // handles +/-inf and nan
    for (int i = 0;
         i < numit ||
           GEOGRAPHICLIB_PANIC("Convergence failure in Math::tauf");
         ++i) {
      T taupa = taupf(tau, es),
        dtau = (taup - taupa) * (1 + e2m * sq(tau)) /
        ( e2m * hypot(T(1), tau) * hypot(T(1), taupa) );
      tau += dtau;
      if (!(fabs(dtau) >= stol))
        break;
    }
    return tau;
  }

  template<typename T> T Math::hypot3(T x, T y, T z) {
#if __cplusplus < 201703L || GEOGRAPHICLIB_PRECISION == 4
    return sqrt(x*x + y*y + z*z);
#else
    return hypot(x, y, z);
#endif
  }

  template<typename T> T Math::NaN() {
#if defined(_MSC_VER)
    return numeric_limits<T>::has_quiet_NaN ?
      numeric_limits<T>::quiet_NaN() :
      (numeric_limits<T>::max)();
#else
    return numeric_limits<T>::has_quiet_NaN ?
      numeric_limits<T>::quiet_NaN() :
      numeric_limits<T>::max();
#endif
  }

  template<typename T> T Math::infinity() {
#if defined(_MSC_VER)
    return numeric_limits<T>::has_infinity ?
        numeric_limits<T>::infinity() :
        (numeric_limits<T>::max)();
#else
    return numeric_limits<T>::has_infinity ?
      numeric_limits<T>::infinity() :
      numeric_limits<T>::max();
#endif
    }

  /// \cond SKIP
  // Instantiate
#define GEOGRAPHICLIB_MATH_INSTANTIATE(T)                                  \
  template T    GEOGRAPHICLIB_EXPORT Math::sum          <T>(T, T, T&);     \
  template T    GEOGRAPHICLIB_EXPORT Math::AngNormalize <T>(T);            \
  template T    GEOGRAPHICLIB_EXPORT Math::AngDiff      <T>(T, T, T&);     \
  template T    GEOGRAPHICLIB_EXPORT Math::AngRound     <T>(T);            \
  template void GEOGRAPHICLIB_EXPORT Math::sincosd      <T>(T, T&, T&);    \
  template void GEOGRAPHICLIB_EXPORT Math::sincosde     <T>(T, T, T&, T&); \
  template T    GEOGRAPHICLIB_EXPORT Math::sind         <T>(T);            \
  template T    GEOGRAPHICLIB_EXPORT Math::cosd         <T>(T);            \
  template T    GEOGRAPHICLIB_EXPORT Math::tand         <T>(T);            \
  template T    GEOGRAPHICLIB_EXPORT Math::atan2d       <T>(T, T);         \
  template T    GEOGRAPHICLIB_EXPORT Math::atand        <T>(T);            \
  template T    GEOGRAPHICLIB_EXPORT Math::eatanhe      <T>(T, T);         \
  template T    GEOGRAPHICLIB_EXPORT Math::taupf        <T>(T, T);         \
  template T    GEOGRAPHICLIB_EXPORT Math::tauf         <T>(T, T);         \
  template T    GEOGRAPHICLIB_EXPORT Math::hypot3       <T>(T, T, T);      \
  template T    GEOGRAPHICLIB_EXPORT Math::NaN          <T>();             \
  template T    GEOGRAPHICLIB_EXPORT Math::infinity     <T>();

  // Instantiate with the standard floating type
  GEOGRAPHICLIB_MATH_INSTANTIATE(float)
  GEOGRAPHICLIB_MATH_INSTANTIATE(double)
#if GEOGRAPHICLIB_HAVE_LONG_DOUBLE
  // Instantiate if long double is distinct from double
  GEOGRAPHICLIB_MATH_INSTANTIATE(long double)
#endif
#if GEOGRAPHICLIB_PRECISION > 3
  // Instantiate with the high precision type
  GEOGRAPHICLIB_MATH_INSTANTIATE(Math::real)
#endif

#undef GEOGRAPHICLIB_MATH_INSTANTIATE

  // Also we need int versions for Utility::nummatch
  template int GEOGRAPHICLIB_EXPORT Math::NaN     <int>();
  template int GEOGRAPHICLIB_EXPORT Math::infinity<int>();
  /// \endcond

} // namespace GeographicLib
