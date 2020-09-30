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
 * @brief  accelerated power method for fast eigenvalue and eigenvector
 * computation
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/dllexport.h>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <random>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace gtsam {

using Sparse = Eigen::SparseMatrix<double>;

/* ************************************************************************* */
/// MINIMUM EIGENVALUE COMPUTATIONS

// Template argument Operator just needs multiplication operator
template <class Operator>
class PowerMethod {
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
 public:
  // Const reference to an externally-held matrix whose minimum-eigenvalue we
  // want to compute
  const Operator &A_;

  const int dim_;  // dimension of Matrix A

  size_t nrIterations_;  // number of iterations

 private:
  double ritzValues_;   // all Ritz eigenvalues
  Vector ritzVectors_;  // all Ritz eigenvectors

 public:
  // Constructor
  explicit PowerMethod(const Operator &A, const Vector &initial)
      : A_(A), dim_(A.rows()), nrIterations_(0) {
    Vector x0;
    x0 = initial.isZero(0) ? Vector::Random(dim_) : initial;
    x0.normalize();

    // initialize Ritz eigen values
    ritzValues_ = 0.0;

    // initialize Ritz eigen vectors
    ritzVectors_.resize(dim_, 1);
    ritzVectors_.setZero();

    ritzVectors_.col(0) = update(x0);
    perturb();
  }

  Vector update(const Vector &x) const {
    Vector y = A_ * x;
    y.normalize();
    return y;
  }

  Vector update() const { return update(ritzVectors_); }

  void updateRitz(const Vector &ritz) { ritzVectors_ = ritz; }

  Vector getRitz() { return ritzVectors_; }

  void perturb() {
    // generate a 0.03*||x_0||_2 as stated in David's paper
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937 generator(seed);
    std::uniform_real_distribution<double> uniform01(0.0, 1.0);

    int n = dim_;
    Vector disturb;
    disturb.resize(n);
    disturb.setZero();
    for (int i = 0; i < n; ++i) {
      disturb(i) = uniform01(generator);
    }
    disturb.normalize();

    Vector x0 = ritzVectors_;
    double magnitude = x0.norm();
    ritzVectors_ = x0 + 0.03 * magnitude * disturb;
  }

  // Perform power iteration on a single Ritz value
  // Updates ritzValues_
  bool iterateOne(double tol) {
    const Vector x = ritzVectors_;
    double theta = x.transpose() * A_ * x;

    // store the Ritz eigen value
    ritzValues_ = theta;

    const Vector diff = A_ * x - theta * x;
    double error = diff.norm();
    return error < tol;
  }

  size_t nrIterations() { return nrIterations_; }

  int compute(int maxit, double tol) {
    // Starting
    int nrConverged = 0;

    for (int i = 0; i < maxit; i++) {
      nrIterations_ += 1;
      ritzVectors_ = update();
      nrConverged = iterateOne(tol);
      if (nrConverged) break;
    }

    return std::min(1, nrConverged);
  }

  double eigenvalues() { return ritzValues_; }

  Vector eigenvectors() { return ritzVectors_; }
};

template <class Operator>
class AcceleratedPowerMethod : public PowerMethod<Operator> {
  double beta_ = 0;  // a Polyak momentum term

  Vector previousVector_;  // store previous vector

 public:
  // Constructor
  explicit AcceleratedPowerMethod(const Operator &A, const Vector &initial)
      : PowerMethod<Operator>(A, initial) {
    Vector x0 = initial;
    // initialize ritz vector
    x0 = x0.isZero(0) ? Vector::Random(PowerMethod<Operator>::dim_) : x0;
    Vector x00 = Vector::Random(PowerMethod<Operator>::dim_);
    x0.normalize();
    x00.normalize();

    // initialize Ritz eigen vector and previous vector
    previousVector_ = update(x0, x00, beta_);
    this->updateRitz(update(previousVector_, x0, beta_));
    this->perturb();

    // set beta
    Vector init_resid = this->getRitz();
    const double up = init_resid.transpose() * this->A_ * init_resid;
    const double down = init_resid.transpose().dot(init_resid);
    const double mu = up / down;
    beta_ = mu * mu / 4;
    setBeta();
  }

  Vector update(const Vector &x1, const Vector &x0, const double beta) const {
    Vector y = this->A_ * x1 - beta * x0;
    y.normalize();
    return y;
  }

  Vector update() const {
    Vector y = update(this->ritzVectors_, previousVector_, beta_);
    previousVector_ = this->ritzVectors_;
    return y;
  }

  /// Tuning the momentum beta using the Best Heavy Ball algorithm in Ref(3)
  void setBeta() {
    if (PowerMethod<Operator>::dim_ < 10) return;
    double maxBeta = beta_;
    size_t maxIndex;
    std::vector<double> betas = {2 / 3 * maxBeta, 0.99 * maxBeta, maxBeta,
                                 1.01 * maxBeta, 1.5 * maxBeta};

    Matrix tmpRitzVectors;
    tmpRitzVectors.resize(PowerMethod<Operator>::dim_, 10);
    tmpRitzVectors.setZero();
    for (size_t i = 0; i < 10; i++) {
      for (size_t k = 0; k < betas.size(); ++k) {
        for (size_t j = 1; j < 10; j++) {
          if (j < 2) {
            Vector x0 = Vector::Random(PowerMethod<Operator>::dim_);
            Vector x00 = Vector::Random(PowerMethod<Operator>::dim_);
            tmpRitzVectors.col(0) = update(x0, x00, betas[k]);
            tmpRitzVectors.col(1) = update(tmpRitzVectors.col(0), x0, betas[k]);
          } else {
            tmpRitzVectors.col(j) = update(tmpRitzVectors.col(j - 1),
                                           tmpRitzVectors.col(j - 2), betas[k]);
          }
          const Vector x = tmpRitzVectors.col(j);
          const double up = x.transpose() * this->A_ * x;
          const double down = x.transpose().dot(x);
          const double mu = up / down;
          if (mu * mu / 4 > maxBeta) {
            maxIndex = k;
            maxBeta = mu * mu / 4;
            break;
          }
        }
      }
    }

    beta_ = betas[maxIndex];
  }
};

}  // namespace gtsam
