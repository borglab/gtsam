/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   PowerMethod.h
 * @date   Sept 2020
 * @author Jing Wu
 * @brief  Power method for fast eigenvalue and eigenvector
 * computation
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <random>
#include <vector>
#include <optional>

namespace gtsam {

using Sparse = Eigen::SparseMatrix<double>;

/**
 * \brief Compute maximum Eigenpair with power method
 *
 * References :
 * 1) G. Golub and C. V. Loan, Matrix Computations, 3rd ed. Baltimore, Johns
 * Hopkins University Press, 1996, pp.405-411
 * 2) Rosen, D. and Carlone, L., 2017, September. Computational
 * enhancements for certifiably correct SLAM. In Proceedings of the
 * International Conference on Intelligent Robots and Systems.
 * 3) Yulun Tian and Kasra Khosoussi and David M. Rosen and Jonathan P. How,
 * 2020, Aug, Distributed Certifiably Correct Pose-Graph Optimization, Arxiv
 * 4) C. de Sa, B. He, I. Mitliagkas, C. Ré, and P. Xu, “Accelerated
 * stochastic power iteration,” in Proc. Mach. Learn. Res., no. 84, 2018, pp.
 * 58–67
 *
 * It performs the following iteration: \f$ x_{k+1} = A * x_k  \f$
 * where A is the aim matrix we want to get eigenpair of, x is the
 * Ritz vector
 *
 * Template argument Operator just needs multiplication operator
 *
 */
template <class Operator>
class PowerMethod {
 protected:
  /**
   * Const reference to an externally-held matrix whose minimum-eigenvalue we
   * want to compute
   */
  const Operator &A_;

  const int dim_;  // dimension of Matrix A

  size_t nrIterations_;  // number of iterations

  double ritzValue_;   // Ritz eigenvalue
  Vector ritzVector_;  // Ritz eigenvector

 public:
  /// @name Standard Constructors
  /// @{

  /// Construct from the aim matrix and intial ritz vector
  explicit PowerMethod(const Operator &A,
                       const std::optional<Vector> initial = {})
      : A_(A), dim_(A.rows()), nrIterations_(0) {
    Vector x0;
    x0 = initial ? *initial : Vector::Random(dim_);
    x0.normalize();

    // initialize Ritz eigen value
    ritzValue_ = 0.0;

    // initialize Ritz eigen vector
    ritzVector_ = powerIteration(x0);
  }

  /**
   * Run power iteration to get ritzVector with previous ritzVector x, and
   * return A * x / || A * x ||
   */
  Vector powerIteration(const Vector &x) const {
    Vector y = A_ * x;
    y.normalize();
    return y;
  }

  /**
   * Run power iteration to get ritzVector with previous ritzVector x, and
   * return A * x / || A * x ||
   */  
  Vector powerIteration() const { return powerIteration(ritzVector_); }

  /**
   * After Perform power iteration on a single Ritz value, check if the Ritz
   * residual for the current Ritz pair is less than the required convergence
   * tol, return true if yes, else false
   */
  bool converged(double tol) const {
    const Vector x = ritzVector_;
    // store the Ritz eigen value
    const double ritzValue = x.dot(A_ * x);
    const double error = (A_ * x - ritzValue * x).norm();
    return error < tol;
  }

  /// Return the number of iterations
  size_t nrIterations() const { return nrIterations_; }

  /**
   * Start the power/accelerated iteration, after performing the
   * power/accelerated iteration, calculate the ritz error, repeat this
   * operation until the ritz error converge. If converged return true, else
   * false.
   */
  bool compute(size_t maxIterations, double tol) {
    // Starting
    bool isConverged = false;

    for (size_t i = 0; i < maxIterations && !isConverged; i++) {
      ++nrIterations_;
      // update the ritzVector after power iteration
      ritzVector_ = powerIteration();
      // update the ritzValue 
      ritzValue_ = ritzVector_.dot(A_ * ritzVector_);
      isConverged = converged(tol);
    }

    return isConverged;
  }

  /// Return the eigenvalue
  double eigenvalue() const { return ritzValue_; }

  /// Return the eigenvector
  Vector eigenvector() const { return ritzVector_; }
};

}  // namespace gtsam
