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

#include <gtsam/nonlinear/internal/gamma.h>
#include <gtsam/nonlinear/internal/halley.h>
#include <gtsam/nonlinear/internal/utils.h>

#include <algorithm>

// TODO(Varun) remove
#include <boost/math/distributions/gamma.hpp>

namespace gtsam {

namespace internal {

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

  // TODO
  //  Get an initial guess (https://dl.acm.org/doi/abs/10.1145/22721.23109)
  //  T guess = find_inverse_gamma<T>(a, p, 1 - p);
  bool has_10_digits = false;
  boost::math::policies::policy<> pol;
  T guess = boost::math::detail::find_inverse_gamma<T>(a, p, 1 - p, pol,
                                                       &has_10_digits);

  T lower = LIM<T>::min();
  if (guess <= lower) {
    guess = LIM<T>::min();
  }

  // TODO
  //  Number of Halley iterations
  //  The default used in Boost is 200
  //  uint_fast16_t max_iter = 200;

  // The number of digits to converge to.
  // This is an arbitrary number,
  // but Boost does more sophisticated things
  // using the first derivative.
  // unsigned digits = 40;

  // // Perform Halley iteration for root-finding to get a more refined answer
  // guess = halley_iterate(gamma_p_inverse_func<T>(a, p, false), guess, lower,
  //                        LIM<T>::max(), digits, max_iter);
  unsigned digits =
      boost::math::policies::digits<T, boost::math::policies::policy<>>();
  if (digits < 30) {
    digits *= 2;
    digits /= 3;
  } else {
    digits /= 2;
    digits -= 1;
  }
  if ((a < T(0.125)) && (fabs(boost::math::gamma_p_derivative(a, guess, pol)) >
                         1 / sqrt(boost::math::tools::epsilon<T>())))
    digits =
        boost::math::policies::digits<T, boost::math::policies::policy<>>() - 2;
  //
  // Go ahead and iterate:
  //
  std::uintmax_t max_iter = boost::math::policies::get_max_root_iterations<
      boost::math::policies::policy<>>();
  guess = boost::math::tools::halley_iterate(
      boost::math::detail::gamma_p_inverse_func<
          T, boost::math::policies::policy<>>(a, p, false),
      guess, lower, boost::math::tools::max_value<T>(), digits, max_iter);

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
