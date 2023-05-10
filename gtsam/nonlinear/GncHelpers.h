/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    GncHelpers.h
 * @brief   Helper functions for use with the GncOptimizer
 * @author  Varun Agrawal
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>

namespace gtsam {

/// Template type for numeric limits
template <class T>
using LIM = std::numeric_limits<T>;

template <typename T>
using return_t =
    typename std::conditional<std::is_integral<T>::value, double, T>::type;

template <typename... T>
using common_t = typename std::common_type<T...>::type;

template <typename... T>
using common_return_t = return_t<common_t<T...>>;

/// Check if integer is odd
constexpr bool is_odd(const long long int x) noexcept { return (x & 1U) != 0; }

/// Templated check for NaN
template <typename T>
constexpr bool is_nan(const T x) noexcept {
  return x != x;
}

/// @brief Gauss-Legendre quadrature: 50 points
static const long double gauss_legendre_50_points[50] = {
    -0.03109833832718887611232898966595L, 0.03109833832718887611232898966595L,
    -0.09317470156008614085445037763960L, 0.09317470156008614085445037763960L,
    -0.15489058999814590207162862094111L, 0.15489058999814590207162862094111L,
    -0.21600723687604175684728453261710L, 0.21600723687604175684728453261710L,
    -0.27628819377953199032764527852113L, 0.27628819377953199032764527852113L,
    -0.33550024541943735683698825729107L, 0.33550024541943735683698825729107L,
    -0.39341431189756512739422925382382L, 0.39341431189756512739422925382382L,
    -0.44980633497403878914713146777838L, 0.44980633497403878914713146777838L,
    -0.50445814490746420165145913184914L, 0.50445814490746420165145913184914L,
    -0.55715830451465005431552290962580L, 0.55715830451465005431552290962580L,
    -0.60770292718495023918038179639183L, 0.60770292718495023918038179639183L,
    -0.65589646568543936078162486400368L, 0.65589646568543936078162486400368L,
    -0.70155246870682225108954625788366L, 0.70155246870682225108954625788366L,
    -0.74449430222606853826053625268219L, 0.74449430222606853826053625268219L,
    -0.78455583290039926390530519634099L, 0.78455583290039926390530519634099L,
    -0.82158207085933594835625411087394L, 0.82158207085933594835625411087394L,
    -0.85542976942994608461136264393476L, 0.85542976942994608461136264393476L,
    -0.88596797952361304863754098246675L, 0.88596797952361304863754098246675L,
    -0.91307855665579189308973564277166L, 0.91307855665579189308973564277166L,
    -0.93665661894487793378087494727250L, 0.93665661894487793378087494727250L,
    -0.95661095524280794299774564415662L, 0.95661095524280794299774564415662L,
    -0.97286438510669207371334410460625L, 0.97286438510669207371334410460625L,
    -0.98535408404800588230900962563249L, 0.98535408404800588230900962563249L,
    -0.99403196943209071258510820042069L, 0.99403196943209071258510820042069L,
    -0.99886640442007105018545944497422L, 0.99886640442007105018545944497422L};

/// @brief Gauss-Legendre quadrature: 50 weights
static const long double gauss_legendre_50_weights[50] = {
    0.06217661665534726232103310736061L, 0.06217661665534726232103310736061L,
    0.06193606742068324338408750978083L, 0.06193606742068324338408750978083L,
    0.06145589959031666375640678608392L, 0.06145589959031666375640678608392L,
    0.06073797084177021603175001538481L, 0.06073797084177021603175001538481L,
    0.05978505870426545750957640531259L, 0.05978505870426545750957640531259L,
    0.05860084981322244583512243663085L, 0.05860084981322244583512243663085L,
    0.05718992564772838372302931506599L, 0.05718992564772838372302931506599L,
    0.05555774480621251762356742561227L, 0.05555774480621251762356742561227L,
    0.05371062188899624652345879725566L, 0.05371062188899624652345879725566L,
    0.05165570306958113848990529584010L, 0.05165570306958113848990529584010L,
    0.04940093844946631492124358075143L, 0.04940093844946631492124358075143L,
    0.04695505130394843296563301363499L, 0.04695505130394843296563301363499L,
    0.04432750433880327549202228683039L, 0.04432750433880327549202228683039L,
    0.04152846309014769742241197896407L, 0.04152846309014769742241197896407L,
    0.03856875661258767524477015023639L, 0.03856875661258767524477015023639L,
    0.03545983561514615416073461100098L, 0.03545983561514615416073461100098L,
    0.03221372822357801664816582732300L, 0.03221372822357801664816582732300L,
    0.02884299358053519802990637311323L, 0.02884299358053519802990637311323L,
    0.02536067357001239044019487838544L, 0.02536067357001239044019487838544L,
    0.02178024317012479298159206906269L, 0.02178024317012479298159206906269L,
    0.01811556071348939035125994342235L, 0.01811556071348939035125994342235L,
    0.01438082276148557441937890892732L, 0.01438082276148557441937890892732L,
    0.01059054838365096926356968149924L, 0.01059054838365096926356968149924L,
    0.00675979919574540150277887817799L, 0.00675979919574540150277887817799L,
    0.00290862255315514095840072434286L, 0.00290862255315514095840072434286L};

namespace internal {

template <typename T>
class IncompleteGamma {
  /// 50 point Gauss-Legendre quadrature values
  static constexpr T quadrature_inp_vals(const T lb, const T ub,
                                         const int counter) noexcept {
    return (ub - lb) * gauss_legendre_50_points[counter] / T(2) +
           (ub + lb) / T(2);
  }

  /// 50 point Gauss-Legendre quadrature weights
  static constexpr T quadrature_weight_vals(const T lb, const T ub,
                                            const int counter) noexcept {
    return (ub - lb) * gauss_legendre_50_weights[counter] / T(2);
  }

  static constexpr T quadrature_fn(const T x, const T a,
                                   const T lg_term) noexcept {
    return exp(-x + (a - T(1)) * log(x) - lg_term);
  }

  static constexpr T quadrature_lb(const T a, const T z) noexcept {
    // break integration into ranges
    return a > T(1000)  ? std::max(T(0), std::min(z, a) - 11 * sqrt(a))
           : a > T(800) ? std::max(T(0), std::min(z, a) - 11 * sqrt(a))
           : a > T(500) ? std::max(T(0), std::min(z, a) - 10 * sqrt(a))
           : a > T(300) ? std::max(T(0), std::min(z, a) - 10 * sqrt(a))
           : a > T(100) ? std::max(T(0), std::min(z, a) - 9 * sqrt(a))
           : a > T(90)  ? std::max(T(0), std::min(z, a) - 9 * sqrt(a))
           : a > T(70)  ? std::max(T(0), std::min(z, a) - 8 * sqrt(a))
           : a > T(50)  ? std::max(T(0), std::min(z, a) - 7 * sqrt(a))
           : a > T(40)  ? std::max(T(0), std::min(z, a) - 6 * sqrt(a))
           : a > T(30)  ? std::max(T(0), std::min(z, a) - 5 * sqrt(a))
                        : std::max(T(0), std::min(z, a) - 4 * sqrt(a));
  }

  static constexpr T quadrature_ub(const T a, const T z) noexcept {
    return a > T(1000)  ? std::min(z, a + 10 * sqrt(a))
           : a > T(800) ? std::min(z, a + 10 * sqrt(a))
           : a > T(500) ? std::min(z, a + 9 * sqrt(a))
           : a > T(300) ? std::min(z, a + 9 * sqrt(a))
           : a > T(100) ? std::min(z, a + 8 * sqrt(a))
           : a > T(90)  ? std::min(z, a + 8 * sqrt(a))
           : a > T(70)  ? std::min(z, a + 7 * sqrt(a))
           : a > T(50)  ? std::min(z, a + 6 * sqrt(a))
                        : std::min(z, a + 5 * sqrt(a));
  }

  static constexpr T quadrature(const T a, const T z) noexcept {
    T lb = quadrature_lb(a, z);
    T ub = quadrature_ub(a, z);
    T lg_term = lgamma(a);
    T value = quadrature_fn(quadrature_inp_vals(lb, ub, 49), a, lg_term) *
              quadrature_weight_vals(lb, ub, 49);
    for (size_t counter = 48; counter >= 0; counter--) {
      value += quadrature_fn(quadrature_inp_vals(lb, ub, counter), a, lg_term) *
               quadrature_weight_vals(lb, ub, counter);
    }
    return value;
  }

  /**
   * @brief Reverse continued fraction expansion
   * See: https://functions.wolfram.com/GammaBetaErf/Gamma2/10/0003/
   *
   * @param a Degrees of freedom
   * @param z
   * @param depth
   * @return constexpr T
   */
  static constexpr T cf_2_recur(const T a, const T z,
                                const int depth) noexcept {
    if (depth < 100) {
      return (1 + (depth - 1) * 2 - a + z) +
             depth * (a - depth) / cf_2_recur(a, z, depth + 1);
    } else {
      return 1 + (depth - 1) * 2 - a + z;
    }
  }

  /**
   * @brief Lower (regularized) incomplete gamma function
   *
   * @param a Degrees of freedom
   * @param z
   * @return constexpr T
   */
  static constexpr T continued_fraction_2(const T a, const T z) noexcept {
    return T(1.0) - exp(a * log(z) - z - lgamma(a)) / cf_2_recur(a, z, 1);
  }

  /**
   * @brief Continued fraction expansion of Gamma function
   * See: http://functions.wolfram.com/GammaBetaErf/Gamma2/10/0009/
   *
   * @param a Degrees of freedom
   * @param z
   * @param depth
   * @return constexpr T
   */
  static constexpr T cf_1_recur(const T a, const T z,
                                const int depth) noexcept {
    if (depth < 55) {
      T cf_coef = is_odd(depth) ? -(a - 1 + T(depth + 1) / T(2)) * z
                                : T(depth) / T(2) * z;
      return (a + depth - 1) + cf_coef / cf_1_recur(a, z, depth + 1);
    } else {
      return (a + depth - 1);
    }
  }

  /**
   * @brief Lower (regularized) incomplete gamma function
   *
   * @param a Degrees of freedom
   * @param z
   * @return constexpr T
   */
  static constexpr T continued_fraction_1(const T a, const T z) noexcept {
    return exp(a * log(z) - z - lgamma(a)) / cf_1_recur(a, z, 1);
  }

 public:
  /**
   * @brief Compute the CDF for the Gamma distribution.
   *
   * @param a Degrees of freedom
   * @param z
   * @return constexpr T
   */
  static constexpr T compute(const T a, const T z) noexcept {
    if (is_nan(a) || is_nan(z)) {  // NaN check
      return LIM<T>::quiet_NaN();
    } else if (a < T(0)) {
      return LIM<T>::quiet_NaN();
    } else if (LIM<T>::min() > z) {
      return T(0);
    } else if (LIM<T>::min() > a) {
      return T(1);
    } else if (a < T(10) && z - a < T(10)) {
      return continued_fraction_1(a, z);
    } else if (a < T(10) || z / a > T(3)) {
      return continued_fraction_2(a, z);
    } else {
      return quadrature(a, z);
    }
  }
};

}  // namespace internal

/**
 * Compile-time regularized lower incomplete gamma function
 *
 * @param a a real-valued, non-negative input.
 * @param x a real-valued, non-negative input.
 *
 * @return the regularized lower incomplete gamma function evaluated at (\c a,
 * \c x), \f[ \frac{\gamma(a,x)}{\Gamma(a)} = \frac{1}{\Gamma(a)} \int_0^x
 * t^{a-1} \exp(-t) dt \f] When \c a is not too large, the value is computed
 * using the continued fraction representation of the upper incomplete gamma
 * function, \f$ \Gamma(a,x) \f$, using \f[ \Gamma(a,x) = \Gamma(a) -
 * \dfrac{x^a\exp(-x)}{a - \dfrac{ax}{a + 1 + \dfrac{x}{a + 2 - \dfrac{(a+1)x}{a
 * + 3 + \dfrac{2x}{a + 4 - \ddots}}}}} \f] where \f$ \gamma(a,x) \f$ and \f$
 * \Gamma(a,x) \f$ are connected via \f[ \frac{\gamma(a,x)}{\Gamma(a)} +
 * \frac{\Gamma(a,x)}{\Gamma(a)} = 1 \f] When \f$ a > 10 \f$, a 50-point
 * Gauss-Legendre quadrature scheme is employed.
 */
template <typename T1, typename T2>
constexpr common_return_t<T1, T2> incomplete_gamma(const T1 a,
                                                   const T2 x) noexcept {
  using TC = common_return_t<T1, T2>;
  return internal::IncompleteGamma<TC>::compute(static_cast<TC>(a),
                                                static_cast<TC>(x));
}

namespace internal {

template <typename T>
class IncompleteGammaInverse {
  /**
   * @brief Compute an initial value for the inverse gamma function which is
   * then iteratively updated.
   *
   * @param a Degrees of freedom
   * @param p
   * @return constexpr T
   */
  static constexpr T initial_val(const T a, const T p) noexcept {
    if (a > T(1)) {
      // Inverse gamma function initial value when a > 1.0
      const T t_val = p > T(0.5) ? sqrt(-2 * log(T(1) - p)) : sqrt(-2 * log(p));
      const T sgn_term = p > T(0.5) ? T(-1) : T(1);
      const T initial_val_1 =
          t_val -
          (T(2.515517L) + T(0.802853L) * t_val + T(0.010328L) * t_val * t_val) /
              (T(1) + T(1.432788L) * t_val + T(0.189269L) * t_val * t_val +
               T(0.001308L) * t_val * t_val * t_val);
      const T signed_initial_val_1 = sgn_term * initial_val_1;

      return std::max(
          T(1e-04),
          a * pow(T(1) - T(1) / (9 * a) - signed_initial_val_1 / (3 * sqrt(a)),
                  3));
    } else {
      // Inverse gamma function initial value when a <= 1.0
      T t_val = T(1) - T(0.253) * a - T(0.12) * a * a;
      if (p < t_val) {
        return pow(p / t_val, T(1) / a);
      } else {
        return T(1) - log(T(1) - (p - t_val) / (T(1) - t_val));
      }
    }
  }

  /**
   * @brief Compute the error value `f(x)` which we can use for root-finding.
   *
   * @param value
   * @param a Degrees of freedom
   * @param p
   * @return constexpr T
   */
  static constexpr T err_val(const T value, const T a, const T p) noexcept {
    return (incomplete_gamma(a, value) - p);
  }

  /**
   * @brief Derivative of the incomplete gamma function w.r.t. value
   *
   * @param value
   * @param a Degrees of freedom
   * @param log_val
   * @return constexpr T
   */
  static constexpr T first_derivative(const T value, const T a,
                                      const T lg_val) noexcept {
    return (exp(-value + (a - T(1)) * log(value) - lg_val));
  }

  /**
   * @brief Second derivative of the incomplete gamma function w.r.t. value
   *
   * @param value
   * @param a Degrees of freedom
   * @param derivative
   * @return constexpr T
   */
  static constexpr T second_derivative(const T value, const T a,
                                       const T derivative) noexcept {
    return (derivative * ((a - T(1)) / value - T(1)));
  }

  /**
   * @brief Compute \f[ \frac{f(x_n)}{f'(x_n)} \f] as part
   * of the update denominator.
   *
   * @param value
   * @param a Degrees of freedom
   * @param p
   * @param derivative
   * @return constexpr T
   */
  static constexpr T ratio_val_1(const T value, const T a, const T p,
                                 const T derivative) noexcept {
    return (err_val(value, a, p) / derivative);
  }

  /**
   * @brief Compute \f[ \frac{f''(x_n)}{f'(x_n)} \f] as part
   * of the update denominator.
   *
   * @param value
   * @param a Degrees of freedom
   * @param derivative
   * @return constexpr T
   */
  static constexpr T ratio_val_2(const T value, const T a,
                                 const T derivative) noexcept {
    return (second_derivative(value, a, derivative) / derivative);
  }

  /**
   * @brief Halley's method update delta
   *
   * @param ratio_val_1
   * @param ratio_val_2
   * @return constexpr T
   */
  static constexpr T halley(const T ratio_val_1, const T ratio_val_2) noexcept {
    return (ratio_val_1 /
            std::max(T(0.8), std::min(T(1.2), T(1) - T(0.5) * ratio_val_1 *
                                                         ratio_val_2)));
  }

  /**
   * @brief Compute the iterative solution for the
   * incomplete inverse gamma function.
   *
   * @param initial_val Initial value guess
   * @param a Degrees of freedom
   * @param p
   * @param lg_val
   * @return constexpr T
   */
  static constexpr T find_root(const T initial_val, const T a, const T p,
                               const T lg_val) noexcept {
    const int GAMMA_INV_MAX_ITER = 35;
    T x = initial_val;
    T derivative = first_derivative(initial_val, a, lg_val);
    for (size_t counter = 1; counter <= GAMMA_INV_MAX_ITER; counter++) {
      T direc = halley(ratio_val_1(x, a, p, derivative),
                       ratio_val_2(x, a, derivative));
      derivative = first_derivative(x, a, lg_val);
      x = x - direc;
    }
    return x - halley(ratio_val_1(x, a, p, derivative),
                      ratio_val_2(x, a, derivative));
  }

 public:
  /**
   * @brief Compute the percent point function for the Gamma distribution.
   *
   * @param a Degrees of freedom
   * @param p
   * @return constexpr T
   */
  static constexpr T compute(const T a, const T p) noexcept {
    // Perform checks on the input and return the corresponding best answer
    if (isnan(a) || isnan(p)) {  // NaN check
      return LIM<T>::quiet_NaN();
    } else if (LIM<T>::min() > p) {  // Check lower bound
      return T(0);
    } else if (p > T(1)) {  // Check upper bound
      return LIM<T>::quiet_NaN();
    } else if (LIM<T>::min() > abs(T(1) - p)) {
      return LIM<T>::infinity();
    } else if (LIM<T>::min() > a) {  // Check lower bound for degrees of freedom
      return T(0);
    } else {
      return find_root(initial_val(a, p), a, p, lgamma(a));
    }
  }
};

}  // namespace internal

/**
 * Compile-time inverse incomplete gamma function
 *
 * Compute the value \f$ x \f$
 * such that \f[ f(x) := \frac{\gamma(a,x)}{\Gamma(a)} - p \f] equal to zero,
 * for a given \c p.
 *
 * We find this root using Halley's method:
 * \f[ x_{n+1} = x_n - \frac{f(x_n)/f'(x_n)}{1 - 0.5 \frac{f(x_n)}{f'(x_n)}
 * \frac{f''(x_n)}{f'(x_n)} } \f] where
 * \f[ \frac{\partial}{\partial x} \left(\frac{\gamma(a,x)}{\Gamma(a)}\right) =
 * \frac{1}{\Gamma(a)} x^{a-1} \exp(-x) \f] \f[ \frac{\partial^2}{\partial x^2}
 * \left(\frac{\gamma(a,x)}{\Gamma(a)}\right) = \frac{1}{\Gamma(a)} x^{a-1}
 * \exp(-x) \left( \frac{a-1}{x} - 1 \right) \f]
 *
 * @param a The degrees of freedom for the gamma distribution.
 * @param p The quantile value for computing the percent point function.
 *
 * @return Computes the inverse incomplete gamma function.
 */
template <typename T1, typename T2>
constexpr common_return_t<T1, T2> incomplete_gamma_inv(const T1 a,
                                                       const T2 p) noexcept {
  using TC = common_return_t<T1, T2>;
  return internal::IncompleteGammaInverse<TC>::compute(static_cast<TC>(a),
                                                       static_cast<TC>(p));
}

/**
 * @brief Compute the quantile function of the Chi squared distribution.
 *
 * @param dofs Degrees of freedom
 * @param alpha Quantile value
 * @return constexpr double
 */
constexpr double chi_squared_quantile(const size_t dofs, const double alpha) {
  // The quantile function of the Chi-squared distribution is the quantile of
  // the specific (inverse) incomplete Gamma distribution
  return 2 * incomplete_gamma_inv(dofs * 0.5, alpha);
}

}  // namespace gtsam
