/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Gamma.h
 * @brief   Gamma and Gamma Inverse functions
 *
 * A lot of this code has been picked up from
 * https://www.boost.org/doc/libs/1_83_0/boost/math/special_functions/detail/igamma_inverse.hpp
 *
 * @author  Varun Agrawal
 */

#pragma once

#include <gtsam/nonlinear/internal/Utils.h>

#include <boost/math/special_functions/gamma.hpp>
#include <boost/math/special_functions/trunc.hpp>

namespace gtsam {

namespace internal {

template <class T>
inline constexpr T log_max_value() {
  return log(LIM<T>::max());
}

/**
 * @brief Upper gamma fraction for integer a
 *
 * @param a
 * @param x
 * @param pol
 * @param pderivative
 * @return template <class T, class Policy>
 */
template <class T, class Policy>
inline T finite_gamma_q(T a, T x, Policy const& pol, T* pderivative = 0) {
  // Calculates normalised Q when a is an integer:
  T e = exp(-x);
  T sum = e;
  if (sum != 0) {
    T term = sum;
    for (unsigned n = 1; n < a; ++n) {
      term /= n;
      term *= x;
      sum += term;
    }
  }
  if (pderivative) {
    *pderivative = e * pow(x, a) /
                   boost::math::unchecked_factorial<T>(std::trunc(T(a - 1)));
  }
  return sum;
}

/**
 * @brief Upper gamma fraction for half integer a
 *
 * @tparam T
 * @tparam Policy
 * @param a
 * @param x
 * @param p_derivative
 * @param pol
 * @return T
 */
template <class T, class Policy>
T finite_half_gamma_q(T a, T x, T* p_derivative, const Policy& pol) {
  // Calculates normalised Q when a is a half-integer:
  T e = boost::math::erfc(sqrt(x), pol);
  if ((e != 0) && (a > 1)) {
    T term = exp(-x) / sqrt(M_PI * x);
    term *= x;
    static const T half = T(1) / 2;
    term /= half;
    T sum = term;
    for (unsigned n = 2; n < a; ++n) {
      term /= n - half;
      term *= x;
      sum += term;
    }
    e += sum;
    if (p_derivative) {
      *p_derivative = 0;
    }
  } else if (p_derivative) {
    // We'll be dividing by x later, so calculate derivative * x:
    *p_derivative = sqrt(x) * exp(-x) / sqrt(M_PI);
  }
  return e;
}

/**
 * @brief Incomplete gamma functions follow
 *
 * @tparam T
 */
template <class T>
struct upper_incomplete_gamma_fract {
 private:
  T z, a;
  int k;

 public:
  typedef std::pair<T, T> result_type;

  upper_incomplete_gamma_fract(T a1, T z1) : z(z1 - a1 + 1), a(a1), k(0) {}

  result_type operator()() {
    ++k;
    z += 2;
    return result_type(k * (a - k), z);
  }
};

template <class T>
inline T upper_gamma_fraction(T a, T z, T eps) {
  // Multiply result by z^a * e^-z to get the full
  // upper incomplete integral.  Divide by tgamma(z)
  // to normalise.
  upper_incomplete_gamma_fract<T> f(a, z);
  return 1 / (z - a + 1 + boost::math::tools::continued_fraction_a(f, eps));
}

/**
 * @brief Main incomplete gamma entry point, handles all four incomplete
 * gamma's:
 *
 * @tparam T
 * @tparam Policy
 * @param a
 * @param x
 * @param normalised
 * @param invert
 * @param pol
 * @param p_derivative
 * @return T
 */
template <class T, class Policy>
T gamma_incomplete_imp(T a, T x, bool normalised, bool invert,
                       const Policy& pol, T* p_derivative) {
  if (a <= 0) {
    throw std::runtime_error(
        "Argument a to the incomplete gamma function must be greater than "
        "zero");
  }
  if (x < 0) {
    throw std::runtime_error(
        "Argument x to the incomplete gamma function must be >= 0");
  }

  typedef typename boost::math::lanczos::lanczos<T, Policy>::type lanczos_type;

  T result = 0;  // Just to avoid warning C4701: potentially uninitialized local
                 // variable 'result' used

  // max_factorial value for long double is 170 in Boost
  if (a >= 170 && !normalised) {
    //
    // When we're computing the non-normalized incomplete gamma
    // and a is large the result is rather hard to compute unless
    // we use logs.  There are really two options - if x is a long
    // way from a in value then we can reliably use methods 2 and 4
    // below in logarithmic form and go straight to the result.
    // Otherwise we let the regularized gamma take the strain
    // (the result is unlikely to underflow in the central region anyway)
    // and combine with lgamma in the hopes that we get a finite result.
    //
    if (invert && (a * 4 < x)) {
      // This is method 4 below, done in logs:
      result = a * log(x) - x;
      if (p_derivative) *p_derivative = exp(result);
      result += log(upper_gamma_fraction(
          a, x, boost::math::policies::get_epsilon<T, Policy>()));
    } else if (!invert && (a > 4 * x)) {
      // This is method 2 below, done in logs:
      result = a * log(x) - x;
      if (p_derivative) *p_derivative = exp(result);
      T init_value = 0;
      result += log(
          boost::math::detail::lower_gamma_series(a, x, pol, init_value) / a);
    } else {
      result = gamma_incomplete_imp(a, x, true, invert, pol, p_derivative);
      if (result == 0) {
        if (invert) {
          // Try http://functions.wolfram.com/06.06.06.0039.01
          result = 1 + 1 / (12 * a) + 1 / (288 * a * a);
          result = log(result) - a + (a - 0.5f) * log(a) + log(sqrt(2 * M_PI));
          if (p_derivative) *p_derivative = exp(a * log(x) - x);
        } else {
          // This is method 2 below, done in logs, we're really outside the
          // range of this method, but since the result is almost certainly
          // infinite, we should probably be OK:
          result = a * log(x) - x;
          if (p_derivative) *p_derivative = exp(result);
          T init_value = 0;
          result += log(
              boost::math::detail::lower_gamma_series(a, x, pol, init_value) /
              a);
        }
      } else {
        result = log(result) + boost::math::lgamma(a, pol);
      }
    }
    if (result > log_max_value<T>()) {
      throw std::overflow_error(
          "gamma_incomplete_imp: result is larger than log of max value");
    }

    return exp(result);
  }

  assert((p_derivative == nullptr) || normalised);

  bool is_int, is_half_int;
  bool is_small_a = (a < 30) && (a <= x + 1) && (x < log_max_value<T>());
  if (is_small_a) {
    T fa = floor(a);
    is_int = (fa == a);
    is_half_int = is_int ? false : (fabs(fa - a) == 0.5f);
  } else {
    is_int = is_half_int = false;
  }

  int eval_method;

  if (is_int && (x > 0.6)) {
    // calculate Q via finite sum:
    invert = !invert;
    eval_method = 0;
  } else if (is_half_int && (x > 0.2)) {
    // calculate Q via finite sum for half integer a:
    invert = !invert;
    eval_method = 1;
  } else if ((x < boost::math::tools::root_epsilon<T>()) && (a > 1)) {
    eval_method = 6;
  } else if ((x > 1000) && ((a < x) || (fabs(a - 50) / x < 1))) {
    // calculate Q via asymptotic approximation:
    invert = !invert;
    eval_method = 7;
  } else if (x < T(0.5)) {
    //
    // Changeover criterion chosen to give a changeover at Q ~ 0.33
    //
    if (T(-0.4) / log(x) < a) {
      eval_method = 2;
    } else {
      eval_method = 3;
    }
  } else if (x < T(1.1)) {
    //
    // Changeover here occurs when P ~ 0.75 or Q ~ 0.25:
    //
    if (x * 0.75f < a) {
      eval_method = 2;
    } else {
      eval_method = 3;
    }
  } else {
    //
    // Begin by testing whether we're in the "bad" zone
    // where the result will be near 0.5 and the usual
    // series and continued fractions are slow to converge:
    //
    bool use_temme = false;
    if (normalised && std::numeric_limits<T>::is_specialized && (a > 20)) {
      T sigma = fabs((x - a) / a);
      if ((a > 200) && (boost::math::policies::digits<T, Policy>() <= 113)) {
        //
        // This limit is chosen so that we use Temme's expansion
        // only if the result would be larger than about 10^-6.
        // Below that the regular series and continued fractions
        // converge OK, and if we use Temme's method we get increasing
        // errors from the dominant erfc term as it's (inexact) argument
        // increases in magnitude.
        //
        if (20 / a > sigma * sigma) use_temme = true;
      } else if (boost::math::policies::digits<T, Policy>() <= 64) {
        // Note in this zone we can't use Temme's expansion for
        // types longer than an 80-bit real:
        // it would require too many terms in the polynomials.
        if (sigma < 0.4) use_temme = true;
      }
    }
    if (use_temme) {
      eval_method = 5;
    } else {
      //
      // Regular case where the result will not be too close to 0.5.
      //
      // Changeover here occurs at P ~ Q ~ 0.5
      // Note that series computation of P is about x2 faster than continued
      // fraction calculation of Q, so try and use the CF only when really
      // necessary, especially for small x.
      //
      if (x - (1 / (3 * x)) < a) {
        eval_method = 2;
      } else {
        eval_method = 4;
        invert = !invert;
      }
    }
  }

  switch (eval_method) {
    case 0: {
      result = finite_gamma_q(a, x, pol, p_derivative);
      if (!normalised) result *= boost::math::tgamma(a, pol);
      break;
    }
    case 1: {
      result =
          boost::math::detail::finite_half_gamma_q(a, x, p_derivative, pol);
      if (!normalised) result *= boost::math::tgamma(a, pol);
      if (p_derivative && (*p_derivative == 0))
        *p_derivative = boost::math::detail::regularised_gamma_prefix(
            a, x, pol, lanczos_type());
      break;
    }
    case 2: {
      // Compute P:
      result = normalised ? boost::math::detail::regularised_gamma_prefix(
                                a, x, pol, lanczos_type())
                          : boost::math::detail::full_igamma_prefix(a, x, pol);
      if (p_derivative) *p_derivative = result;
      if (result != 0) {
        //
        // If we're going to be inverting the result then we can
        // reduce the number of series evaluations by quite
        // a few iterations if we set an initial value for the
        // series sum based on what we'll end up subtracting it from
        // at the end.
        // Have to be careful though that this optimization doesn't
        // lead to spurious numeric overflow.  Note that the
        // scary/expensive overflow checks below are more often
        // than not bypassed in practice for "sensible" input
        // values:
        //
        T init_value = 0;
        bool optimised_invert = false;
        if (invert) {
          init_value = (normalised ? 1 : boost::math::tgamma(a, pol));
          if (normalised || (result >= 1) ||
              (LIM<T>::max() * result > init_value)) {
            init_value /= result;
            if (normalised || (a < 1) || (LIM<T>::max() / a > init_value)) {
              init_value *= -a;
              optimised_invert = true;
            } else
              init_value = 0;
          } else
            init_value = 0;
        }
        result *=
            boost::math::detail::lower_gamma_series(a, x, pol, init_value) / a;
        if (optimised_invert) {
          invert = false;
          result = -result;
        }
      }
      break;
    }
    case 3: {
      // Compute Q:
      invert = !invert;
      T g;
      result = boost::math::detail::tgamma_small_upper_part(
          a, x, pol, &g, invert, p_derivative);
      invert = false;
      if (normalised) result /= g;
      break;
    }
    case 4: {
      // Compute Q:
      result = normalised ? boost::math::detail::regularised_gamma_prefix(
                                a, x, pol, lanczos_type())
                          : boost::math::detail::full_igamma_prefix(a, x, pol);
      if (p_derivative) *p_derivative = result;
      if (result != 0)
        result *= upper_gamma_fraction(
            a, x, boost::math::policies::get_epsilon<T, Policy>());
      break;
    }
    case 5: {
      //
      // Use compile time dispatch to the appropriate
      // Temme asymptotic expansion.  This may be dead code
      // if T does not have numeric limits support, or has
      // too many digits for the most precise version of
      // these expansions, in that case we'll be calling
      // an empty function.
      //
      typedef typename boost::math::policies::precision<T, Policy>::type
          precision_type;

      typedef std::integral_constant<int, precision_type::value <= 0     ? 0
                                          : precision_type::value <= 53  ? 53
                                          : precision_type::value <= 64  ? 64
                                          : precision_type::value <= 113 ? 113
                                                                         : 0>
          tag_type;

      result = boost::math::detail::igamma_temme_large(
          a, x, pol, static_cast<tag_type const*>(nullptr));
      if (x >= a) invert = !invert;
      if (p_derivative)
        *p_derivative = boost::math::detail::regularised_gamma_prefix(
            a, x, pol, lanczos_type());
      break;
    }
    case 6: {
      // x is so small that P is necessarily very small too,
      // use
      // http://functions.wolfram.com/GammaBetaErf/GammaRegularized/06/01/05/01/01/
      if (!normalised)
        result = pow(x, a) / (a);
      else {
        try {
          result = pow(x, a) / boost::math::tgamma(a + 1, pol);
        } catch (const std::overflow_error&) {
          result = 0;
        }
      }
      result *= 1 - a * x / (a + 1);
      if (p_derivative)
        *p_derivative = boost::math::detail::regularised_gamma_prefix(
            a, x, pol, lanczos_type());
      break;
    }
    case 7: {
      // x is large,
      // Compute Q:
      result = normalised ? boost::math::detail::regularised_gamma_prefix(
                                a, x, pol, lanczos_type())
                          : boost::math::detail::full_igamma_prefix(a, x, pol);
      if (p_derivative) *p_derivative = result;
      result /= x;
      if (result != 0)
        result *= boost::math::detail::incomplete_tgamma_large_x(a, x, pol);
      break;
    }
  }

  if (normalised && (result > 1)) result = 1;
  if (invert) {
    T gam = normalised ? 1 : boost::math::tgamma(a, pol);
    result = gam - result;
  }
  if (p_derivative) {
    //
    // Need to convert prefix term to derivative:
    //
    if ((x < 1) && (LIM<T>::max() * x < *p_derivative)) {
      // overflow, just return an arbitrarily large value:
      *p_derivative = LIM<T>::max() / 2;
    }

    *p_derivative /= x;
  }

  return result;
}

/**
 * @brief Functional to compute the gamma inverse.
 * Mainly used with Halley iteration.
 *
 * @tparam T
 */
template <class T>
struct gamma_p_inverse_func {
  gamma_p_inverse_func(T a_, T p_, bool inv) : a(a_), p(p_), invert(inv) {
    /*
    If p is too near 1 then P(x) - p suffers from cancellation
    errors causing our root-finding algorithms to "thrash", better
    to invert in this case and calculate Q(x) - (1-p) instead.

    Of course if p is *very* close to 1, then the answer we get will
    be inaccurate anyway (because there's not enough information in p)
    but at least we will converge on the (inaccurate) answer quickly.
    */
    if (p > T(0.9)) {
      p = 1 - p;
      invert = !invert;
    }
  }

  std::tuple<T, T, T> operator()(const T& x) const {
    // Calculate P(x) - p and the first two derivates, or if the invert
    // flag is set, then Q(x) - q and it's derivatives.
    T f, f1;
    T ft;
    boost::math::policies::policy<> pol;
    f = static_cast<T>(
        internal::gamma_incomplete_imp(a, x, true, invert, pol, &ft));
    f1 = ft;
    T f2;
    T div = (a - x - 1) / x;
    f2 = f1;

    if (fabs(div) > 1) {
      if (internal::LIM<T>::max() / fabs(div) < f2) {
        // overflow:
        f2 = -internal::LIM<T>::max() / 2;
      } else {
        f2 *= div;
      }
    } else {
      f2 *= div;
    }

    if (invert) {
      f1 = -f1;
      f2 = -f2;
    }

    return std::make_tuple(static_cast<T>(f - p), f1, f2);
  }

 private:
  T a, p;
  bool invert;
};

}  // namespace internal
}  // namespace gtsam
