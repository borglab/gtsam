/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Chebyshev.h
 * @brief Chebyshev basis decompositions
 * @author Varun Agrawal
 * @date July 4, 2020
 */

#pragma once

#include <gtsam/base/Manifold.h>
#include <gtsam/basis/Basis.h>

#include <gtsam/3rdparty/Eigen/unsupported/Eigen/KroneckerProduct>

namespace gtsam {

/**
 * Basis of Chebyshev polynomials of the second kind.
 * https://en.wikipedia.org/wiki/Chebyshev_polynomials#Second_kind
 * These are typically denoted with the symbol U_n, where n is the degree.
 * The parameter N is the number of coefficients, i.e., N = n+1.
 * In contrast to the templates in Chebyshev2, the classes below specify
 * basis functions, weighted combinations of which are used to approximate
 * functions. In this sense, they are like the sines and cosines of the Fourier
 * basis.
 */
struct Chebyshev2Basis : Basis<Chebyshev2Basis> {
  using Weights = Eigen::Matrix<double, 1, -1 /*1xN*/>;
  using Parameters = Eigen::Matrix<double, -1, 1 /*Nx1*/>;

  /**
   *  Evaluate Chebyshev Weights on [-1,1] at any x up to order N-1 (N values)
   */
  static Weights CalculateWeights(size_t N, double x);
};
// Chebyshev2Basis

/**
 * Basis of Chebyshev polynomials of the first kind
 * https://en.wikipedia.org/wiki/Chebyshev_polynomials#First_kind
 * These are typically denoted with the symbol T_n, where n is the degree.
 * The parameter N is the number of coefficients, i.e., N = n+1.
 */
struct Chebyshev1Basis : Basis<Chebyshev1Basis> {
  using Weights = Eigen::Matrix<double, 1, -1 /*1xN*/>;
  using Parameters = Eigen::Matrix<double, -1, 1 /*Nx1*/>;

  /**
   *  Evaluate Chebyshev Weights on [-1,1] at x up to order N-1 (N values)
   */
  static Weights CalculateWeights(size_t N, double x);

  /// Create a Chebyshev derivative function of Parameters
  class Derivative {
   protected:
    // From Wikipedia we have D[T_n(x),x] = n*U_{n-1}(x)
    // I.e. the derivative fo a first kind cheb is just a second kind cheb
    // So, we define a second kind basis here of order N-1
    // Note that it has one less weight:
    typename Chebyshev2Basis::Weights weights_;

    size_t N_;

   public:
    Derivative(size_t N, double x)
        : weights_(Chebyshev2Basis::CalculateWeights(N - 1, x)), N_(N) {
      // after the above init, weights_ contains the U_{n-1} values for n=1..N-1
      // Note there is no value for n=0. But we need n*U{n-1}, so correct:
      for (size_t n = 1; n < N; n++) {
        weights_(n - 1) *= double(n);
      }
    }

    double operator()(const Parameters &c) {
      // The Parameters pertain to 1st kind chebs up to order N-1
      // But of course the first one (order 0) is constant, so omit that weight
      return (weights_ * c.block(1, 0, N_ - 1, 1))(0);
    }
  };
};
// Chebyshev1Basis

}  // namespace gtsam
