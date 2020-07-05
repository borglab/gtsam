/**
 * @file Chebyshev2.h
 * @brief Chebyshev parameterizations on Chebyshev points of second kind.
 *        This is different from Chebyshev.h since it leverage ideas from
 *        pseudo-spectral optimization, i.e. we don't decompose into basis
 *        functions, rather estimate function parameters that enforce function
 *        nodes at Chebyshev points.
 * @author Varun Agrawal
 * @date July 4, 2020
 */

#pragma once

#include <gtsam/base/Manifold.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/basis/Basis.h>

#include <boost/function.hpp>

namespace gtsam {

/**
 * Chebyshev Interpolation on Chebyshev points of the second kind
 * Note that N here, the #points, is one less than N from Trefethen00book
 * (pg.42)
 */
class Chebyshev2 : public Basis<Chebyshev2> {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:
  using Base = Basis<Chebyshev2>;
  using Weights = Eigen::Matrix<double, /*1xN*/ 1, -1>;
  using Parameters = Eigen::Matrix<double, /*Nx1*/ -1, 1>;
  using DiffMatrix = Eigen::Matrix<double, /*NxN*/ -1, -1>;

  /**
   * A matrix of M*N values at the Chebyshev points, where M is the dimension of
   * T template argument T: the type you would like to EvaluationFunctor using
   * polynomial interpolation template argument N: the number of Chebyshev
   * points of the second kind
   */
  template <typename T>
  struct ParameterMatrix {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // First dimension is: traits<T>::dimension
    typedef Eigen::Matrix<double, /*TxN*/ -1, -1> type;
  };

  /// Specific Chebyshev point
  static double Point(size_t N, int j) {
    assert(j >= 0 && size_t(j) < N);
    const double dtheta = M_PI / (N > 1 ? (N - 1) : 1);
    // We add -PI so that we get values ordered from -1 to +1
    // sin(- M_PI_2 + dtheta*j); also works
    return cos(-M_PI + dtheta * j);
  }

  /// Specific Chebyshev point, within [a,b] interval
  static double Point(size_t N, int j, double a, double b) {
    assert(j >= 0 && size_t(j) < N);
    const double dtheta = M_PI / (N - 1);
    // We add -PI so that we get values ordered from -1 to +1
    return a + (b - a) * (1. + cos(-M_PI + dtheta * j)) / 2;
  }

  /// All Chebyshev points
  static Vector Points(size_t N) {
    Vector points(N);
    for (size_t j = 0; j < N; j++) points(j) = Point(N, j);
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
   *
   * Please refer to Trefethen13chapters p 35, formula 5.13
   */
  static Weights CalculateWeights(size_t N, double x, double a = -1,
                                  double b = 1);

  /**
   *  Evaluate derivative of barycentric weights.
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
  static Matrix matrix(
      boost::function<Eigen::Matrix<double, M, 1>(double)> f,  //
      size_t N, double a = -1, double b = 1) {
    Matrix Xmat(M, N);
    for (size_t j = 0; j < N; j++) Xmat.col(j) = f(Point(N, j, a, b));
    return Xmat;
  }
};
// \ Chebyshev2

}  // namespace gtsam
