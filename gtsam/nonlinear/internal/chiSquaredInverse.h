/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    chiSquaredInverse.h
 * @brief   This file contains an implementation of the Chi Squared inverse
 * function, which is implemented similar to Boost with additional template
 * parameter helpers.
 *
 * A lot of this code has been picked up from
 * https://www.boost.org/doc/libs/1_83_0/boost/math/special_functions/detail/igamma_inverse.hpp
 * https://www.boost.org/doc/libs/1_83_0/boost/math/tools/roots.hpp
 *
 * @author  Varun Agrawal
 */

#pragma once

#include <gtsam/nonlinear/internal/Gamma.h>
#include <gtsam/nonlinear/internal/Utils.h>

#include <algorithm>

// TODO(Varun) remove
#include <boost/math/tools/roots.hpp>

namespace gtsam {

namespace internal {

/**
 * @brief Polynomial evaluation with runtime size.
 *
 * @tparam T
 * @tparam U
 */
template <class T, class U>
inline U evaluate_polynomial(const T* poly, U const& z, std::size_t count) {
  assert(count > 0);
  U sum = static_cast<U>(poly[count - 1]);
  for (int i = static_cast<int>(count) - 2; i >= 0; --i) {
    sum *= z;
    sum += static_cast<U>(poly[i]);
  }
  return sum;
}

/**
 * @brief Computation of the Incomplete Gamma Function Ratios and their Inverse.
 *
 * Reference:
 * ARMIDO R. DIDONATO and ALFRED H. MORRIS, JR.
 * ACM Transactions on Mathematical Software, Vol. 12, No. 4,
 * December 1986, Pages 377-393.
 *
 * See equation 32.
 *
 * @tparam T
 * @param p
 * @param q
 * @return T
 */
template <class T>
T find_inverse_s(T p, T q) {
  T t;
  if (p < T(0.5)) {
    t = sqrt(-2 * log(p));
  } else {
    t = sqrt(-2 * log(q));
  }
  static const double a[4] = {3.31125922108741, 11.6616720288968,
                              4.28342155967104, 0.213623493715853};
  static const double b[5] = {1, 6.61053765625462, 6.40691597760039,
                              1.27364489782223, 0.3611708101884203e-1};
  T s = t - internal::evaluate_polynomial(a, t, 4) /
                internal::evaluate_polynomial(b, t, 5);
  if (p < T(0.5)) s = -s;
  return s;
}

/**
 * @brief Computation of the Incomplete Gamma Function Ratios and their Inverse.
 *
 * Reference:
 * ARMIDO R. DIDONATO and ALFRED H. MORRIS, JR.
 * ACM Transactions on Mathematical Software, Vol. 12, No. 4,
 * December 1986, Pages 377-393.
 *
 * See equation 34.
 *
 * @tparam T
 * @param a
 * @param x
 * @param N
 * @param tolerance
 * @return T
 */
template <class T>
T didonato_SN(T a, T x, unsigned N, T tolerance = 0) {
  T sum = 1;
  if (N >= 1) {
    T partial = x / (a + 1);
    sum += partial;
    for (unsigned i = 2; i <= N; ++i) {
      partial *= x / (a + i);
      sum += partial;
      if (partial < tolerance) break;
    }
  }
  return sum;
}

/**
 * @brief Compute the initial inverse gamma value guess.
 *
 * We use the implementation in this paper:
 * Computation of the Incomplete Gamma Function Ratios and their Inverse
 * ARMIDO R. DIDONATO and ALFRED H. MORRIS, JR.
 * ACM Transactions on Mathematical Software, Vol. 12, No. 4,
 * December 1986, Pages 377-393.
 *
 * @tparam T
 * @param a
 * @param p
 * @param q
 * @param p_has_10_digits
 * @return T
 */
template <class T>
T find_inverse_gamma(T a, T p, T q, bool* p_has_10_digits) {
  T result;
  *p_has_10_digits = false;

  // TODO(Varun) replace with egamma_v<double> in C++20
  //  Euler-Mascheroni constant
  double euler = 0.577215664901532860606512090082402431042159335939923598805;

  if (a == 1) {
    result = -log(q);
  } else if (a < 1) {
    T g = std::tgamma(a);
    T b = q * g;

    if ((b > T(0.6)) || ((b >= T(0.45)) && (a >= T(0.3)))) {
      // DiDonato & Morris Eq 21:
      //
      // There is a slight variation from DiDonato and Morris here:
      // the first form given here is unstable when p is close to 1,
      // making it impossible to compute the inverse of Q(a,x) for small
      // q.  Fortunately the second form works perfectly well in this case.
      T u;
      if ((b * q > T(1e-8)) && (q > T(1e-5))) {
        u = pow(p * g * a, 1 / a);
      } else {
        u = exp((-q / a) - euler);
      }
      result = u / (1 - (u / (a + 1)));

    } else if ((a < 0.3) && (b >= 0.35)) {
      // DiDonato & Morris Eq 22:
      T t = exp(-euler - b);
      T u = t * exp(t);
      result = t * exp(u);

    } else if ((b > 0.15) || (a >= 0.3)) {
      // DiDonato & Morris Eq 23:
      T y = -log(b);
      T u = y - (1 - a) * log(y);
      result = y - (1 - a) * log(u) - log(1 + (1 - a) / (1 + u));

    } else if (b > 0.1) {
      // DiDonato & Morris Eq 24:
      T y = -log(b);
      T u = y - (1 - a) * log(y);
      result = y - (1 - a) * log(u) -
               log((u * u + 2 * (3 - a) * u + (2 - a) * (3 - a)) /
                   (u * u + (5 - a) * u + 2));

    } else {
      // DiDonato & Morris Eq 25:
      T y = -log(b);
      T c1 = (a - 1) * log(y);
      T c1_2 = c1 * c1;
      T c1_3 = c1_2 * c1;
      T c1_4 = c1_2 * c1_2;
      T a_2 = a * a;
      T a_3 = a_2 * a;

      T c2 = (a - 1) * (1 + c1);
      T c3 = (a - 1) * (-(c1_2 / 2) + (a - 2) * c1 + (3 * a - 5) / 2);
      T c4 = (a - 1) * ((c1_3 / 3) - (3 * a - 5) * c1_2 / 2 +
                        (a_2 - 6 * a + 7) * c1 + (11 * a_2 - 46 * a + 47) / 6);
      T c5 = (a - 1) * (-(c1_4 / 4) + (11 * a - 17) * c1_3 / 6 +
                        (-3 * a_2 + 13 * a - 13) * c1_2 +
                        (2 * a_3 - 25 * a_2 + 72 * a - 61) * c1 / 2 +
                        (25 * a_3 - 195 * a_2 + 477 * a - 379) / 12);

      T y_2 = y * y;
      T y_3 = y_2 * y;
      T y_4 = y_2 * y_2;
      result = y + c1 + (c2 / y) + (c3 / y_2) + (c4 / y_3) + (c5 / y_4);

      if (b < 1e-28f) *p_has_10_digits = true;
    }
  } else {
    // DiDonato and Morris Eq 31:
    T s = find_inverse_s(p, q);

    T s_2 = s * s;
    T s_3 = s_2 * s;
    T s_4 = s_2 * s_2;
    T s_5 = s_4 * s;
    T ra = sqrt(a);

    T w = a + s * ra + (s * s - 1) / 3;
    w += (s_3 - 7 * s) / (36 * ra);
    w -= (3 * s_4 + 7 * s_2 - 16) / (810 * a);
    w += (9 * s_5 + 256 * s_3 - 433 * s) / (38880 * a * ra);

    if ((a >= 500) && (fabs(1 - w / a) < 1e-6)) {
      result = w;
      *p_has_10_digits = true;

    } else if (p > 0.5) {
      if (w < 3 * a) {
        result = w;

      } else {
        T D = (std::max)(T(2), T(a * (a - 1)));
        T lg = std::lgamma(a);
        T lb = log(q) + lg;
        if (lb < -D * T(2.3)) {
          // DiDonato and Morris Eq 25:
          T y = -lb;
          T c1 = (a - 1) * log(y);
          T c1_2 = c1 * c1;
          T c1_3 = c1_2 * c1;
          T c1_4 = c1_2 * c1_2;
          T a_2 = a * a;
          T a_3 = a_2 * a;

          T c2 = (a - 1) * (1 + c1);
          T c3 = (a - 1) * (-(c1_2 / 2) + (a - 2) * c1 + (3 * a - 5) / 2);
          T c4 =
              (a - 1) * ((c1_3 / 3) - (3 * a - 5) * c1_2 / 2 +
                         (a_2 - 6 * a + 7) * c1 + (11 * a_2 - 46 * a + 47) / 6);
          T c5 = (a - 1) * (-(c1_4 / 4) + (11 * a - 17) * c1_3 / 6 +
                            (-3 * a_2 + 13 * a - 13) * c1_2 +
                            (2 * a_3 - 25 * a_2 + 72 * a - 61) * c1 / 2 +
                            (25 * a_3 - 195 * a_2 + 477 * a - 379) / 12);

          T y_2 = y * y;
          T y_3 = y_2 * y;
          T y_4 = y_2 * y_2;
          result = y + c1 + (c2 / y) + (c3 / y_2) + (c4 / y_3) + (c5 / y_4);

        } else {
          // DiDonato and Morris Eq 33:
          T u = -lb + (a - 1) * log(w) - log(1 + (1 - a) / (1 + w));
          result = -lb + (a - 1) * log(u) - log(1 + (1 - a) / (1 + u));
        }
      }
    } else {
      T z = w;
      T ap1 = a + 1;
      T ap2 = a + 2;
      if (w < 0.15f * ap1) {
        // DiDonato and Morris Eq 35:
        T v = log(p) + std::lgamma(ap1);
        z = exp((v + w) / a);
        s = std::log1p(z / ap1 * (1 + z / ap2));
        z = exp((v + z - s) / a);
        s = std::log1p(z / ap1 * (1 + z / ap2));
        z = exp((v + z - s) / a);
        s = std::log1p(z / ap1 * (1 + z / ap2 * (1 + z / (a + 3))));
        z = exp((v + z - s) / a);
      }

      if ((z <= 0.01 * ap1) || (z > 0.7 * ap1)) {
        result = z;
        if (z <= T(0.002) * ap1) *p_has_10_digits = true;

      } else {
        // DiDonato and Morris Eq 36:
        T ls = log(didonato_SN(a, z, 100, T(1e-4)));
        T v = log(p) + std::lgamma(ap1);
        z = exp((v + z - ls) / a);
        result = z * (1 - (a * log(z) - z - v + ls) / (a - z));
      }
    }
  }
  return result;
}

template <typename T>
T gamma_p_inv_imp(const T a, const T p) {
  if (is_nan(a) || is_nan(p)) {
    return LIM<T>::quiet_NaN();
    if (a <= T(0)) {
      throw std::runtime_error(
          "Argument a in the incomplete gamma function inverse must be >= 0.");
    }
  } else if (p < T(0) || p > T(1)) {
    throw std::runtime_error(
        "Probability must be in the range [0,1] in the incomplete gamma "
        "function inverse.");
  } else if (p == T(0)) {
    return 0;
  }

  //  Get an initial guess (https://dl.acm.org/doi/abs/10.1145/22721.23109)
  bool has_10_digits = false;
  T guess = find_inverse_gamma<T>(a, p, 1 - p, &has_10_digits);
  if (has_10_digits) {
    return guess;
  }

  T lower = LIM<T>::min();
  if (guess <= lower) {
    guess = LIM<T>::min();
  }

  // The number of digits to converge to.
  // This is an arbitrary but reasonable number,
  // though Boost does more sophisticated things
  // using the first derivative.
  unsigned digits = 25;

  //  Number of Halley iterations
  uintmax_t max_iter = 200;

  // TODO
  // Perform Halley iteration for root-finding to get a more refined answer
  // guess = halley_iterate(gamma_p_inverse_func<T>(a, p, false), guess, lower,
  //                        LIM<T>::max(), digits, max_iter);

  // Go ahead and iterate:
  guess = boost::math::tools::halley_iterate(
      internal::gamma_p_inverse_func<T>(a, p, false), guess, lower,
      LIM<T>::max(), digits, max_iter);

  if (guess == lower) {
    throw std::runtime_error(
        "Expected result known to be non-zero, but is smaller than the "
        "smallest available number.");
  }

  return guess;
}

/**
 * Compile-time check for inverse incomplete gamma function
 *
 * @param a a real-valued, non-negative input.
 * @param p a real-valued input with values in the unit-interval.
 */
template <typename T1, typename T2>
constexpr common_return_t<T1, T2> incomplete_gamma_inv(const T1 a,
                                                       const T2 p) noexcept {
  return internal::gamma_p_inv_imp(static_cast<common_return_t<T1, T2>>(a),
                                   static_cast<common_return_t<T1, T2>>(p));
}

/**
 * @brief Compute the quantile function of the Chi-Squared distribution.
 *
 * @param dofs Degrees of freedom
 * @param alpha Quantile value
 * @return double
 */
double chi_squared_quantile(const double dofs, const double alpha) {
  // The quantile function of the Chi-squared distribution is the quantile of
  // the specific (inverse) incomplete Gamma distribution
  return 2 * incomplete_gamma_inv(dofs / 2, alpha);
}

}  // namespace internal

}  // namespace gtsam
