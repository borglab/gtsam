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

namespace gtsam {

namespace internal {

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
    f = static_cast<T>(boost::math::detail::gamma_incomplete_imp(
        a, x, true, invert, pol, &ft));
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
