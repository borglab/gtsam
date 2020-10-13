/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010-2019, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   AcceleratedPowerMethod.h
 * @date   Sept 2020
 * @author Jing Wu
 * @brief  accelerated power method for fast eigenvalue and eigenvector
 * computation
 */

#pragma once

// #include <gtsam/base/Matrix.h>
// #include <gtsam/base/Vector.h>
#include <gtsam/linear/PowerMethod.h>

namespace gtsam {

using Sparse = Eigen::SparseMatrix<double>;

/* ************************************************************************* */
/// MINIMUM EIGENVALUE COMPUTATIONS

// Template argument Operator just needs multiplication operator
template <class Operator>
class AcceleratedPowerMethod : public PowerMethod<Operator> {
  /**
   * \brief Compute maximum Eigenpair with power method
   *
   * References :
   * 1) Rosen, D. and Carlone, L., 2017, September. Computational
   * enhancements for certifiably correct SLAM. In Proceedings of the
   * International Conference on Intelligent Robots and Systems.
   * 2) Yulun Tian and Kasra Khosoussi and David M. Rosen and Jonathan P. How,
   * 2020, Aug, Distributed Certifiably Correct Pose-Graph Optimization, Arxiv
   * 3) C. de Sa, B. He, I. Mitliagkas, C. Ré, and P. Xu, “Accelerated
   * stochastic power iteration,” in Proc. Mach. Learn. Res., no. 84, 2018, pp.
   * 58–67
   *
   * It performs the following iteration: \f$ x_{k+1} = A * x_k + \beta *
   * x_{k-1} \f$ where A is the certificate matrix, x is the Ritz vector
   *
   */

  double beta_ = 0;  // a Polyak momentum term

  Vector previousVector_;  // store previous vector

 public:
  // Constructor from aim matrix A (given as Matrix or Sparse), optional intial
  // vector as ritzVector
  explicit AcceleratedPowerMethod(
      const Operator &A, const boost::optional<Vector> initial = boost::none)
      : PowerMethod<Operator>(A, initial) {
    Vector x0;
    // initialize ritz vector
    x0 = initial ? Vector::Random(this->dim_) : initial.get();
    Vector x00 = Vector::Random(this->dim_);
    x0.normalize();
    x00.normalize();

    // initialize Ritz eigen vector and previous vector
    previousVector_ = update(x0, x00, beta_);
    this->ritzVector_ = update(previousVector_, x0, beta_);
    this->perturb();

    // set beta
    Vector init_resid = this->ritzVector_;
    const double up = init_resid.transpose() * this->A_ * init_resid;
    const double down = init_resid.transpose().dot(init_resid);
    const double mu = up / down;
    beta_ = mu * mu / 4;
    setBeta();
  }

  // Update the ritzVector with beta and previous two ritzVector
  Vector update(const Vector &x1, const Vector &x0, const double beta) const {
    Vector y = this->A_ * x1 - beta * x0;
    y.normalize();
    return y;
  }

  // Update the ritzVector with beta and previous two ritzVector
  Vector update() const {
    Vector y = update(this->ritzVector_, previousVector_, beta_);
    previousVector_ = this->ritzVector_;
    return y;
  }

  // Tuning the momentum beta using the Best Heavy Ball algorithm in Ref(3)
  void setBeta() {
    double maxBeta = beta_;
    size_t maxIndex;
    std::vector<double> betas = {2 / 3 * maxBeta, 0.99 * maxBeta, maxBeta,
                                 1.01 * maxBeta, 1.5 * maxBeta};

    Matrix R = Matrix::Zero(this->dim_, 10);
    for (size_t i = 0; i < 10; i++) {
      for (size_t k = 0; k < betas.size(); ++k) {
        for (size_t j = 1; j < 10; j++) {
          if (j < 2) {
            Vector x0 = this->ritzVector_;
            Vector x00 = previousVector_;
            R.col(0) = update(x0, x00, betas[k]);
            R.col(1) = update(R.col(0), x0, betas[k]);
          } else {
            R.col(j) = update(R.col(j - 1), R.col(j - 2), betas[k]);
          }
        }
        const Vector x = R.col(9);
        const double up = x.transpose() * this->A_ * x;
        const double down = x.transpose().dot(x);
        const double mu = up / down;
        if (mu * mu / 4 > maxBeta) {
          maxIndex = k;
          maxBeta = mu * mu / 4;
        }
      }
    }
    beta_ = betas[maxIndex];
  }
};

}  // namespace gtsam
