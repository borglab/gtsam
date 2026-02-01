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

#include <Eigen/Dense>

#include <cassert>

namespace gtsam {

double Chebyshev2::Point(size_t N, int j) {
  if (N == 1) return 0.0;
  assert(j >= 0 && size_t(j) < N);
  const double dTheta = M_PI / (N - 1);
  return -cos(dTheta * j);
}

double Chebyshev2::Point(size_t N, int j, double a, double b) {
  if (N == 1) return (a + b) / 2;
  return a + (b - a) * (Point(N, j) + 1.0) / 2.0;
}

Vector Chebyshev2::Points(size_t N) {
  Vector points(N);
  if (N == 1) {
    points(0) = 0.0;
    return points;
  }
  size_t half = N / 2;
  const double dTheta = M_PI / (N - 1);
  for (size_t j = 0; j < half; ++j) {
    double c = cos(j * dTheta);
    points(j) = -c;
    points(N - 1 - j) = c;
  }
  if (N % 2 == 1) {
    points(half) = 0.0;
  }
  return points;
}

Vector Chebyshev2::Points(size_t N, double a, double b) {
  Vector points = Points(N);
  const double T1 = (a + b) / 2, T2 = (b - a) / 2;
  points = T1 + (T2 * points).array();
  return points;
}

namespace {
  // Find the index of the Chebyshev point that coincides with x
  // within the interval [a, b]. If no such point exists, return nullopt.
  std::optional<size_t> coincidentPoint(size_t N, double x, double a, double b, double tol = 1e-12) {
    if (N == 0) return std::nullopt;
    if (N == 1) {
      double mid = (a + b) / 2;
      if (std::abs(x - mid) < tol) return 0;
    } else {
      // Compute normalized value y such that cos(j*dTheta) = y.
      double y = 1.0 - 2.0 * (x - a) / (b - a);
      if (y < -1.0 || y > 1.0) return std::nullopt;
      double dTheta = M_PI / (N - 1);
      double jCandidate = std::acos(y) / dTheta;
      size_t jRounded = static_cast<size_t>(std::round(jCandidate));
      if (std::abs(jCandidate - jRounded) < tol) return jRounded;
    }
    return std::nullopt;
  }

  // Get signed distances from x to all Chebyshev points
  Vector signedDistances(size_t N, double x, double a, double b) {
    Vector result(N);
    const Vector points = Chebyshev2::Points(N, a, b);
    for (size_t j = 0; j < N; j++) {
      const double dj = x - points[j];
      result(j) = dj;
    }
    return result;
  }

  // Helper function to calculate a row of the differentiation matrix, [-1,1] interval
  Vector differentiationMatrixRow(size_t N, const Vector& points, size_t i) {
    Vector row(N);
    const size_t K = N - 1;
    double xi = points(i);
    for (size_t j = 0; j < N; j++) {
      if (i == j) {
        // Diagonal elements
        if (i == 0 || i == K)
          row(j) = (i == 0 ? -1 : 1) * (2.0 * K * K + 1) / 6.0;
        else
          row(j) = -xi / (2.0 * (1.0 - xi * xi));
      }
      else {
        double xj = points(j);
        double ci = (i == 0 || i == K) ? 2. : 1.;
        double cj = (j == 0 || j == K) ? 2. : 1.;
        double t = ((i + j) % 2) == 0 ? 1 : -1;
        row(j) = (ci / cj) * t / (xi - xj);
      }
    }
    return row;
  }
}  // namespace

Weights Chebyshev2::CalculateWeights(size_t N, double x, double a, double b) {
  // We start by getting distances from x to all Chebyshev points
  const Vector distances = signedDistances(N, x, a, b);

  Weights weights(N);
  if (auto j = coincidentPoint(N, x, a, b)) {
    // exceptional case: x coincides with a Chebyshev point
    weights.setZero();
    weights(*j) = 1;
    return weights;
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
  if (auto j = coincidentPoint(N, x, a, b)) {
    // exceptional case: x coincides with a Chebyshev point
    return differentiationMatrixRow(N, Points(N), *j) / ((b - a) / 2.0);
  }
  
  // This section of code computes the derivative of
  // the Barycentric Interpolation weights formula by applying
  // the chain rule on the original formula.
  
  // g and k are multiplier terms which represent the derivatives of
  // the numerator and denominator
  double g = 0, k = 0;
  double w;
  
  const Vector distances = signedDistances(N, x, a, b);
  for (size_t j = 0; j < N; j++) {
    if (j == 0 || j == N - 1) {
      w = 0.5;
    } else {
      w = 1.0;
    }
    
    double t = (j % 2 == 0) ? 1 : -1;
    
    double c = t / distances(j);
    g += w * c;
    k += (w * c / distances(j));
  }
  
  double s = 1;  // changes sign s at every iteration
  double g2 = g * g;
  
  Weights weightDerivatives(N);
  for (size_t j = 0; j < N; j++) {
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
    s *= -1;
  }

  return weightDerivatives;
}

Chebyshev2::DiffMatrix Chebyshev2::DifferentiationMatrix(size_t N) {
  DiffMatrix D(N, N);
  if (N == 1) {
    D(0, 0) = 1;
    return D;
  }

  const Vector points = Points(N);
  for (size_t i = 0; i < N; i++) {
    D.row(i) = differentiationMatrixRow(N, points, i);
  }
  return D;
}

Chebyshev2::DiffMatrix Chebyshev2::DifferentiationMatrix(size_t N, double a, double b) {
  DiffMatrix D(N, N);
  if (N == 1) {
    D(0, 0) = 1;
    return D;
  }

  // Calculate for [-1,1] and scale for [a,b]
  return DifferentiationMatrix(N) / ((b - a) / 2.0);
}

Matrix Chebyshev2::IntegrationMatrix(size_t N) {
  // Obtain the differentiation matrix.
  const Matrix D = DifferentiationMatrix(N);

  // Compute the pseudo-inverse of the differentiation matrix.
  Eigen::JacobiSVD<Matrix> svd(D, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto& S = svd.singularValues();
  Matrix invS = Matrix::Zero(N, N);
  for (size_t i = 0; i < N - 1; ++i) invS(i, i) = 1.0 / S(i);
  Matrix P = svd.matrixV() * invS * svd.matrixU().transpose();

  // Return a version of P that makes sure (P*f)(0) = 0.
  const Weights row0 = P.row(0);
  P.rowwise() -= row0;
  return P;
}

Matrix Chebyshev2::IntegrationMatrix(size_t N, double a, double b) {
  return IntegrationMatrix(N) * (b - a) / 2.0;
}

/*
   Trefethen00book, pg 128, clencurt.m
   Note that N in clencurt.m is 1 less than our N, we call it K below.
   K = N-1;
   theta = pi*(0:K)'/K;
   w = zeros(1,N); ii = 2:K; v = ones(K-1, 1);
   if mod(K,2) == 0
       w(1) = 1/(K^2-1); w(N) = w(1);
       for k=1:K/2-1, v = v-2*cos(2*k*theta(ii))/(4*k^2-1); end
       v = v - cos(K*theta(ii))/(K^2-1);
   else
       w(1) = 1/K^2; w(N) = w(1);
       for k=1:K/2, v = v-2*cos(2*k*theta(ii))/(4*k^2-1); end
   end
   w(ii) = 2*v/K;
*/
Weights Chebyshev2::IntegrationWeights(size_t N) {
  Weights weights(N);
  const size_t K = N - 1,  // number of intervals between N points
    K2 = K * K;

  // Compute endpoint weight.
  weights(0) = 1.0 / (K2 + K % 2 - 1);
  weights(N - 1) = weights(0);

  // Compute up to the middle; mirror symmetry holds.
  const size_t mid = (N - 1) / 2;
  double dTheta = M_PI / K;
  for (size_t i = 1; i <= mid; ++i) {
    double w = (K % 2 == 0) ? 1 - cos(i * M_PI) / (K2 - 1) : 1;
    const size_t last_k = K / 2 + K % 2 - 1;
    const double theta = i * dTheta;
    for (size_t k = 1; k <= last_k; ++k)
      w -= 2.0 * cos(2 * k * theta) / (4 * k * k - 1);
    w *= 2.0 / K;
    weights(i) = w;
    weights(N - 1 - i) = w;
  }
  return weights;
}

Weights Chebyshev2::IntegrationWeights(size_t N, double a, double b) {
  return IntegrationWeights(N) * (b - a) / 2.0;
}

Weights Chebyshev2::DoubleIntegrationWeights(size_t N) {
  // we have w * P, where w are the Clenshaw-Curtis weights and P is the integration matrix
  // But P does not by default return a function starting at zero.
  return Chebyshev2::IntegrationWeights(N) * Chebyshev2::IntegrationMatrix(N);
}

Weights Chebyshev2::DoubleIntegrationWeights(size_t N, double a, double b) {
  return Chebyshev2::IntegrationWeights(N, a, b) * Chebyshev2::IntegrationMatrix(N, a, b);
}

/**
 * Create vector of values at Chebyshev points given scalar-valued function.
 */
Vector Chebyshev2::vector(std::function<double(double)> f, size_t N, double a, double b) {
  Vector fvals(N);
  const Vector points = Points(N, a, b);
  for (size_t j = 0; j < N; j++) fvals(j) = f(points(j));
  return fvals;
}

}  // namespace gtsam
