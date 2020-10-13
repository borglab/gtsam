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

namespace gtsam {

using Sparse = Eigen::SparseMatrix<double>;

/* ************************************************************************* */
/// MINIMUM EIGENVALUE COMPUTATIONS

// Template argument Operator just needs multiplication operator
template <class Operator>
class PowerMethod {
 protected:
  // Const reference to an externally-held matrix whose minimum-eigenvalue we
  // want to compute
  const Operator &A_;

  const int dim_;  // dimension of Matrix A

  size_t nrIterations_;  // number of iterations

  double ritzValue_;   // all Ritz eigenvalues
  Vector ritzVector_;  // all Ritz eigenvectors

 public:
  // Constructor
  explicit PowerMethod(const Operator &A,
                       const boost::optional<Vector> initial = boost::none)
      : A_(A), dim_(A.rows()), nrIterations_(0) {
    Vector x0;
    x0 = initial ? Vector::Random(dim_) : initial.get();
    x0.normalize();

    // initialize Ritz eigen values
    ritzValue_ = 0.0;

    // initialize Ritz eigen vectors
    ritzVector_ = Vector::Zero(dim_);

    ritzVector_.col(0) = update(x0);
    perturb();
  }

  // Update the vector by dot product with A_
  Vector update(const Vector &x) const {
    Vector y = A_ * x;
    y.normalize();
    return y;
  }

  // Update the vector by dot product with A_
  Vector update() const { return update(ritzVector_); }

  // Perturb the initial ritzvector
  void perturb() {
    // generate a 0.03*||x_0||_2 as stated in David's paper
    std::mt19937 rng(42);
    std::uniform_real_distribution<double> uniform01(0.0, 1.0);

    int n = dim_;
    // Vector disturb(n);
    // for (int i = 0; i < n; ++i) {
    //   disturb(i) = uniform01(rng);
    // }
    Vector disturb = Vector::Random(n);
    disturb.normalize();

    Vector x0 = ritzVector_;
    double magnitude = x0.norm();
    ritzVector_ = x0 + 0.03 * magnitude * disturb;
  }

  // Perform power iteration on a single Ritz value
  // Updates ritzValue_
  bool iterateOne(double tol) {
    const Vector x = ritzVector_;
    double theta = x.transpose() * A_ * x;

    // store the Ritz eigen value
    ritzValue_ = theta;

    const Vector diff = A_ * x - theta * x;
    double error = diff.norm();
    return error < tol;
  }

  // Return the number of iterations
  size_t nrIterations() const { return nrIterations_; }

  // Start the iteration until the ritz error converge
  int compute(int maxIterations, double tol) {
    // Starting
    int nrConverged = 0;

    for (int i = 0; i < maxIterations; i++) {
      nrIterations_ += 1;
      ritzVector_ = update();
      nrConverged = iterateOne(tol);
      if (nrConverged) break;
    }

    return std::min(1, nrConverged);
  }

  // Return the eigenvalue
  double eigenvalues() const { return ritzValue_; }

  // Return the eigenvector
  const Vector eigenvectors() const { return ritzVector_; }
};

}  // namespace gtsam
