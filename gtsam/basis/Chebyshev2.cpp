/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Chebyshev2.cpp
 * @brief Chebyshev parameterizations on Chebyshev points of second kind
 * @author Varun Agrawal, Jing Dong, Frank Dellaert
 * @date July 4, 2020
 */

#include <gtsam/basis/Chebyshev2.h>

namespace gtsam {

Weights Chebyshev2::CalculateWeights(size_t N, double x, double a, double b) {
  // Allocate space for weights
  Weights weights(N);

  // We start by getting distances from x to all Chebyshev points
  // as well as getting smallest distance
  Weights distances(N);

  for (size_t j = 0; j < N; j++) {
    const double dj =
        x - Point(N, j, a, b);  // only thing that depends on [a,b]

    if (std::abs(dj) < 1e-10) {
      // exceptional case: x coincides with a Chebyshev point
      weights.setZero();
      weights(j) = 1;
      return weights;
    }
    distances(j) = dj;
  }

  // Beginning of interval, j = 0, x(0) = a
  weights(0) = 0.5 / distances(0);

  // All intermediate points j=1:N-2
  double d = weights(0), s = -1;  // changes sign s at every iteration
  for (size_t j = 1; j < N - 1; j++, s = -s) {
    weights(j) = s / distances(j);
    d += weights(j);
  }

  // End of interval, j = N-1, x(N-1) = b
  weights(N - 1) = 0.5 * s / distances(N - 1);
  d += weights(N - 1);

  // normalize
  return weights / d;
}

Weights Chebyshev2::DerivativeWeights(size_t N, double x, double a, double b) {
  // Allocate space for weights
  Weights weightDerivatives(N);

  // toggle variable so we don't need to use `pow` for -1
  double t = -1;

  // We start by getting distances from x to all Chebyshev points
  // as well as getting smallest distance
  Weights distances(N);

  for (size_t j = 0; j < N; j++) {
    const double dj =
        x - Point(N, j, a, b);  // only thing that depends on [a,b]
    if (std::abs(dj) < 1e-10) {
      // exceptional case: x coincides with a Chebyshev point
      weightDerivatives.setZero();
      // compute the jth row of the differentiation matrix for this point
      double cj = (j == 0 || j == N - 1) ? 2. : 1.;
      for (size_t k = 0; k < N; k++) {
        if (j == 0 && k == 0) {
          // we reverse the sign since we order the cheb points from -1 to 1
          weightDerivatives(k) = -(cj * (N - 1) * (N - 1) + 1) / 6.0;
        } else if (j == N - 1 && k == N - 1) {
          // we reverse the sign since we order the cheb points from -1 to 1
          weightDerivatives(k) = (cj * (N - 1) * (N - 1) + 1) / 6.0;
        } else if (k == j) {
          double xj = Point(N, j);
          double xj2 = xj * xj;
          weightDerivatives(k) = -0.5 * xj / (1 - xj2);
        } else {
          double xj = Point(N, j);
          double xk = Point(N, k);
          double ck = (k == 0 || k == N - 1) ? 2. : 1.;
          t = ((j + k) % 2) == 0 ? 1 : -1;
          weightDerivatives(k) = (cj / ck) * t / (xj - xk);
        }
      }
      return 2 * weightDerivatives / (b - a);
    }
    distances(j) = dj;
  }

  // This section of code computes the derivative of
  // the Barycentric Interpolation weights formula by applying
  // the chain rule on the original formula.

  // g and k are multiplier terms which represent the derivatives of
  // the numerator and denominator
  double g = 0, k = 0;
  double w = 1;

  for (size_t j = 0; j < N; j++) {
    if (j == 0 || j == N - 1) {
      w = 0.5;
    } else {
      w = 1.0;
    }

    t = (j % 2 == 0) ? 1 : -1;

    double c = t / distances(j);
    g += w * c;
    k += (w * c / distances(j));
  }

  double s = 1;  // changes sign s at every iteration
  double g2 = g * g;

  for (size_t j = 0; j < N; j++, s = -s) {
    // Beginning of interval, j = 0, x0 = -1.0 and end of interval, j = N-1,
    // x0 = 1.0
    if (j == 0 || j == N - 1) {
      w = 0.5;
    } else {
      // All intermediate points j=1:N-2
      w = 1.0;
    }
    weightDerivatives(j) = (w * -s / (g * distances(j) * distances(j))) -
                           (w * -s * k / (g2 * distances(j)));
  }

  return weightDerivatives;
}

Chebyshev2::DiffMatrix Chebyshev2::DifferentiationMatrix(size_t N, double a,
                                                         double b) {
  DiffMatrix D(N, N);
  if (N == 1) {
    D(0, 0) = 1;
    return D;
  }

  // toggle variable so we don't need to use `pow` for -1
  double t = -1;

  for (size_t i = 0; i < N; i++) {
    double xi = Point(N, i);
    double ci = (i == 0 || i == N - 1) ? 2. : 1.;
    for (size_t j = 0; j < N; j++) {
      if (i == 0 && j == 0) {
        // we reverse the sign since we order the cheb points from -1 to 1
        D(i, j) = -(ci * (N - 1) * (N - 1) + 1) / 6.0;
      } else if (i == N - 1 && j == N - 1) {
        // we reverse the sign since we order the cheb points from -1 to 1
        D(i, j) = (ci * (N - 1) * (N - 1) + 1) / 6.0;
      } else if (i == j) {
        double xi2 = xi * xi;
        D(i, j) = -xi / (2 * (1 - xi2));
      } else {
        double xj = Point(N, j);
        double cj = (j == 0 || j == N - 1) ? 2. : 1.;
        t = ((i + j) % 2) == 0 ? 1 : -1;
        D(i, j) = (ci / cj) * t / (xi - xj);
      }
    }
  }
  // scale the matrix to the range
  return D / ((b - a) / 2.0);
}

Weights Chebyshev2::IntegrationWeights(size_t N, double a, double b) {
  // Allocate space for weights
  Weights weights(N);
  size_t K = N - 1,  // number of intervals between N points
      K2 = K * K;
  weights(0) = 0.5 * (b - a) / (K2 + K % 2 - 1);
  weights(N - 1) = weights(0);

  size_t last_k = K / 2 + K % 2 - 1;

  for (size_t i = 1; i <= N - 2; ++i) {
    double theta = i * M_PI / K;
    weights(i) = (K % 2 == 0) ? 1 - cos(K * theta) / (K2 - 1) : 1;

    for (size_t k = 1; k <= last_k; ++k)
      weights(i) -= 2 * cos(2 * k * theta) / (4 * k * k - 1);
    weights(i) *= (b - a) / K;
  }

  return weights;
}

}  // namespace gtsam
