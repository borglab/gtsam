/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Chebyshev.cpp
 * @brief Chebyshev basis decompositions
 * @author Varun Agrawal, Jing Dong, Frank Dellaert
 * @date July 4, 2020
 */

#include <gtsam/basis/Chebyshev.h>

namespace gtsam {

/**
 * @brief Scale x from [a, b] to [t1, t2]
 *
 * ((b'-a') * (x - a) / (b - a)) + a'
 *
 * @param x Value to scale to new range.
 * @param a Original lower limit.
 * @param b Original upper limit.
 * @param t1 New lower limit.
 * @param t2 New upper limit.
 * @return double
 */
static double scale(double x, double a, double b, double t1, double t2) {
  return ((t2 - t1) * (x - a) / (b - a)) + t1;
}

Weights Chebyshev1Basis::CalculateWeights(size_t N, double x, double a,
                                          double b) {
  Weights Tx(1, N);

  x = scale(x, a, b, -1, 1);

  Tx(0) = 1;
  Tx(1) = x;
  for (size_t i = 2; i < N; i++) {
    // instead of cos(i*acos(x)), this recurrence is much faster
    Tx(i) = 2 * x * Tx(i - 1) - Tx(i - 2);
  }
  return Tx;
}

Weights Chebyshev1Basis::DerivativeWeights(size_t N, double x, double a,
                                           double b) {
  Weights Ux = Chebyshev2Basis::CalculateWeights(N, x, a, b);
  Weights weights = Weights::Zero(N);
  for (size_t n = 1; n < N; n++) {
    weights(n) = n * Ux(n - 1);
  }
  return weights;
}

Weights Chebyshev2Basis::CalculateWeights(size_t N, double x, double a,
                                          double b) {
  Weights Ux(N);

  x = scale(x, a, b, -1, 1);

  Ux(0) = 1;
  Ux(1) = 2 * x;
  for (size_t i = 2; i < N; i++) {
    // instead of cos(i*acos(x)), this recurrence is much faster
    Ux(i) = 2 * x * Ux(i - 1) - Ux(i - 2);
  }
  return Ux;
}

Weights Chebyshev2Basis::DerivativeWeights(size_t N, double x, double a,
                                           double b) {
  Weights Tx = Chebyshev1Basis::CalculateWeights(N + 1, x, a, b);
  Weights Ux = Chebyshev2Basis::CalculateWeights(N, x, a, b);

  Weights weights(N);

  x = scale(x, a, b, -1, 1);
  if (x == -1 || x == 1) {
    throw std::runtime_error(
        "Derivative of Chebyshev2 Basis does not exist at range limits.");
  }

  for (size_t n = 0; n < N; n++) {
    weights(n) = ((n + 1) * Tx(n + 1) - x * Ux(n)) / (x * x - 1);
  }
  return weights;
}

}  // namespace gtsam
