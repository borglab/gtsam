/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Chebyshev2.h
 * @brief Pseudo-spectral parameterization for Chebyshev polynomials of the
 * second kind.
 *
 * In a pseudo-spectral case, rather than the parameters acting as
 * weights for the bases polynomials (as in Chebyshev2Basis), here the
 * parameters are the *values* at a specific set of points in the interval, the
 * "Chebyshev points". These values uniquely determine the polynomial that
 * interpolates them at the Chebyshev points.
 *
 * This is different from Chebyshev.h since it leverage ideas from
 * pseudo-spectral optimization, i.e. we don't decompose into basis functions,
 * rather estimate function values at the Chebyshev points.
 *
 * Please refer to Agrawal21icra for more details.
 *
 * @author Varun Agrawal, Jing Dong, Frank Dellaert
 * @date July 4, 2020
 */

#pragma once

#include <gtsam/base/Manifold.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/basis/Basis.h>

namespace gtsam {

/**
 * Chebyshev Interpolation on Chebyshev points of the second kind
 * Note that N here, the number of points, is one less than N from
 * 'Approximation Theory and Approximation Practice by L. N. Trefethen (pg.42)'.
 */
class GTSAM_EXPORT Chebyshev2 : public Basis<Chebyshev2> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = Basis<Chebyshev2>;
  using Parameters = Eigen::Matrix<double, /*Nx1*/ -1, 1>;
  using DiffMatrix = Eigen::Matrix<double, /*NxN*/ -1, -1>;

  /**
   * @brief Specific Chebyshev point, within [a,b] interval.
   * Default interval is [-1, 1]
   *
   * @param N The degree of the polynomial
   * @param j The index of the Chebyshev point
   * @param a Lower bound of interval (default: -1)
   * @param b Upper bound of interval (default: 1)
   * @return double
   */
  static double Point(size_t N, int j, double a = -1, double b = 1) {
    assert(j >= 0 && size_t(j) < N);
    const double dtheta = M_PI / (N > 1 ? (N - 1) : 1);
    // We add -PI so that we get values ordered from -1 to +1
    // sin(-M_PI_2 + dtheta*j); also works
    return a + (b - a) * (1. + cos(-M_PI + dtheta * j)) / 2;
  }

  /// All Chebyshev points
  static Vector Points(size_t N) {
    Vector points(N);
    for (size_t j = 0; j < N; j++) {
      points(j) = Point(N, j);
    }
    return points;
  }

  /// All Chebyshev points, within [a,b] interval
  static Vector Points(size_t N, double a, double b) {
    Vector points = Points(N);
    const double T1 = (a + b) / 2, T2 = (b - a) / 2;
    points = T1 + (T2 * points).array();
    return points;
  }

  /**
   * Evaluate Chebyshev Weights on [-1,1] at any x up to order N-1 (N values)
   * These weights implement barycentric interpolation at a specific x.
   * More precisely, f(x) ~ [w0;...;wN] * [f0;...;fN], where the fj are the
   * values of the function f at the Chebyshev points. As such, for a given x we
   * obtain a linear map from parameter vectors f to interpolated values f(x).
   * Optional [a,b] interval can be specified as well.
   */
  static Weights CalculateWeights(size_t N, double x, double a = -1,
                                  double b = 1);

  /**
   * Calculate weights for all x in vector X.
   * Returns M*N matrix where M is the size of the vector X,
   * and N is the number of basis functions.
   *
   * Overriden for Chebyshev2.
   */
  static Matrix WeightMatrix(size_t N, const Vector& X, double a = -1,
                             double b = 1);

  /**
   *  Evaluate derivative of barycentric weights.
   *  This is easy and efficient via the DifferentiationMatrix.
   */
  static Weights DerivativeWeights(size_t N, double x, double a = -1,
                                   double b = 1);

  /// compute D = differentiation matrix, Trefethen00book p.53
  /// when given a parameter vector f of function values at the Chebyshev
  /// points, D*f are the values of f'.
  /// https://people.maths.ox.ac.uk/trefethen/8all.pdf Theorem 8.4
  static DiffMatrix DifferentiationMatrix(size_t N, double a = -1,
                                          double b = 1);

  /**
   *  Evaluate Clenshaw-Curtis integration weights.
   *  Trefethen00book, pg 128, clencurt.m
   *  Note that N in clencurt.m is 1 less than our N
   *  K = N-1;
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
  static Weights IntegrationWeights(size_t N, double a = -1, double b = 1);

  /**
   * Create matrix of values at Chebyshev points given vector-valued function.
   */
  template <size_t M>
  static Matrix matrix(std::function<Eigen::Matrix<double, M, 1>(double)> f,
                       size_t N, double a = -1, double b = 1) {
    Matrix Xmat(M, N);
    for (size_t j = 0; j < N; j++) {
      Xmat.col(j) = f(Point(N, j, a, b));
    }
    return Xmat;
  }
};  // \ Chebyshev2

}  // namespace gtsam
