/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Fourier.h
 * @brief Fourier decomposition, see e.g.
 *        http://mathworld.wolfram.com/FourierSeries.html
 * @author Varun Agrawal, Frank Dellaert
 * @date July 4, 2020
 */

#pragma once

#include <gtsam/basis/Basis.h>

namespace gtsam {

/// Fourier basis
class FourierBasis : public Basis<FourierBasis> {
 public:
  using Parameters = Eigen::Matrix<double, /*Nx1*/ -1, 1>;
  using DiffMatrix = Eigen::Matrix<double, /*NxN*/ -1, -1>;

  /**
   * @brief Evaluate Real Fourier Weights of size N in interval [a, b],
   *        e.g. N=5 yields bases: 1, cos(x), sin(x), cos(2*x), sin(2*x)
   *
   * @param N The degree of the polynomial to use.
   * @param x The point at which to compute the derivaive weights.
   * @return Weights
   */
  static Weights CalculateWeights(size_t N, double x) {
    Weights b(N);
    b[0] = 1;
    for (size_t i = 1, n = 1; i < N; i++) {
      if (i % 2 == 1) {
        b[i] = cos(n * x);
      } else {
        b[i] = sin(n * x);
        n++;
      }
    }
    return b;
  }

  /// Return a zero initialized Parameter matrix.
  static Parameters ParameterMatrix(size_t N) {
    return Parameters::Zero(N);
  }

  /**
   * @brief Evaluate Real Fourier Weights of size N in interval [a, b],
   *        e.g. N=5 yields bases: 1, cos(x), sin(x), cos(2*x), sin(2*x)
   *
   * @param N The degree of the polynomial to use.
   * @param x The point at which to compute the weights.
   * @param a Lower bound of interval.
   * @param b Upper bound of interval.
   * @return Weights
   */
  static Weights CalculateWeights(size_t N, double x, double a, double b) {
    // TODO(Varun) How do we enforce an interval for Fourier series?
    return CalculateWeights(N, x);
  }

  /**
   * Compute D = differentiation matrix.
   * Given coefficients c of a Fourier series c, D*c are the values of c'.
   */
  static DiffMatrix DifferentiationMatrix(size_t N) {
    DiffMatrix D = DiffMatrix::Zero(N, N);
    double k = 1;
    for (size_t i = 1; i < N; i += 2) {
      D(i, i + 1) = k;   // sin'(k*x) = k*cos(k*x)
      D(i + 1, i) = -k;  // cos'(k*x) = -k*sin(k*x)
      k += 1;
    }

    return D;
  }

  /**
   * @brief Get weights at a given x that calculate the derivative.
   *
   * @param N The degree of the polynomial to use.
   * @param x The point at which to compute the derivaive weights.
   * @return Weights
   */
  static Weights DerivativeWeights(size_t N, double x) {
    return CalculateWeights(N, x) * DifferentiationMatrix(N);
  }

  /**
   * @brief Get derivative weights at a given x that calculate the derivative,
  in the interval [a, b].
   *
   * @param N The degree of the polynomial to use.
   * @param x The point at which to compute the derivaive weights.
   * @param a Lower bound of interval.
   * @param b Upper bound of interval.
   * @return Weights
   */
  static Weights DerivativeWeights(size_t N, double x, double a, double b) {
    return CalculateWeights(N, x, a, b) * DifferentiationMatrix(N);
  }

};  // FourierBasis

}  // namespace gtsam
