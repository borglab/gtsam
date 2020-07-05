/**
 * @file Fourier.h
 * @brief Fourier decomposition, see e.g.
 * http://mathworld.wolfram.com/FourierSeries.html
 * @author Varun Agrawal
 * @date July 4, 2020
 */

#pragma once

#include <gtsam/basis/Basis.h>

namespace gtsam {

/// Fourier basis
class FourierBasis : public Basis<FourierBasis> {
 public:
  using Weights = Eigen::Matrix<double, /*1xN*/ 1, -1>;
  using Parameters = Eigen::Matrix<double, /*Nx1*/ -1, 1>;
  using DiffMatrix = Eigen::Matrix<double, /*NxN*/ -1, -1>;

  /**
   *  Evaluate Real Fourier Weights of size N
   *  e.g. N=5 yields bases: 1, cos(x), sin(x), cos(2*x), sin(2*x)
   */
  static Weights CalculateWeights(size_t N, double x) {
    Weights b(N);
    b[0] = 1;
    for (size_t i = 1, n = 1; i < N; i += 2, n++) {
      b[i] = cos(n * x);
      b[i + 1] = sin(n * x);
    }
    return b;
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

  /// Create a Fourier derivative function of Parameters
  class Derivative {
   protected:
    typename FourierBasis::Weights weights_;

    size_t N_;

   public:
    Derivative(size_t N, double x)
        : weights_(FourierBasis::CalculateWeights(N - 1, x)), N_(N) {}

    /// Get weights at a given x that calculate the derivative.
    Weights operator()(const Parameters &c) {
      return CalculateWeights(N_, x) * DifferentiationMatrix(N_);
    }
  };

};  // FourierBasis

}  // namespace gtsam
