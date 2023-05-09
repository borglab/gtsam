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
 * @author Varun Agrawal, Jing Dong, Frank Dellaert
 * @date July 4, 2020
 */

#pragma once

#include <gtsam/base/Manifold.h>
#include <gtsam/basis/Basis.h>

namespace gtsam {

/**
 * Basis of Chebyshev polynomials of the first kind
 * https://en.wikipedia.org/wiki/Chebyshev_polynomials#First_kind
 * These are typically denoted with the symbol T_n, where n is the degree.
 * The parameter N is the number of coefficients, i.e., N = n+1.
 */
struct GTSAM_EXPORT Chebyshev1Basis : Basis<Chebyshev1Basis> {
  using Parameters = Eigen::Matrix<double, -1, 1 /*Nx1*/>;

  Parameters parameters_;

  /**
   * @brief Evaluate Chebyshev Weights on [-1,1] at x up to order N-1 (N values)
   *
   * @param N Degree of the polynomial.
   * @param x Point to evaluate polynomial at.
   * @param a Lower limit of polynomial (default=-1).
   * @param b Upper limit of polynomial (default=1).
   */
  static Weights CalculateWeights(size_t N, double x, double a = -1,
                                  double b = 1);

  /**
   * @brief Evaluate Chebyshev derivative at x.
   * The derivative weights are pre-multiplied to the polynomial Parameters.
   *
   * From Wikipedia we have D[T_n(x),x] = n*U_{n-1}(x)
   * I.e. the derivative fo a first kind cheb is just a second kind cheb
   * So, we define a second kind basis here of order N-1
   * Note that it has one less weight.
   *
   * The Parameters pertain to 1st kind chebs up to order N-1
   * But of course the first one (order 0) is constant, so omit that weight.
   *
   * @param N Degree of the polynomial.
   * @param x Point to evaluate polynomial at.
   * @param a Lower limit of polynomial (default=-1).
   * @param b Upper limit of polynomial (default=1).
   * @return Weights
   */
  static Weights DerivativeWeights(size_t N, double x, double a = -1,
                                   double b = 1);
};  // Chebyshev1Basis

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
struct GTSAM_EXPORT Chebyshev2Basis : Basis<Chebyshev2Basis> {
  using Parameters = Eigen::Matrix<double, -1, 1 /*Nx1*/>;

  /**
   *  Evaluate Chebyshev Weights on [-1,1] at any x up to order N-1 (N values).
   *
   * @param N Degree of the polynomial.
   * @param x Point to evaluate polynomial at.
   * @param a Lower limit of polynomial (default=-1).
   * @param b Upper limit of polynomial (default=1).
   */
  static Weights CalculateWeights(size_t N, double x, double a = -1,
                                  double b = 1);

  /**
   * @brief Evaluate Chebyshev derivative at x.
   *
   * @param N Degree of the polynomial.
   * @param x Point to evaluate polynomial at.
   * @param a Lower limit of polynomial (default=-1).
   * @param b Upper limit of polynomial (default=1).
   * @return Weights
   */
  static Weights DerivativeWeights(size_t N, double x, double a = -1,
                                   double b = 1);
};  // Chebyshev2Basis

}  // namespace gtsam
