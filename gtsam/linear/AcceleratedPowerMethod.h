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

#include <gtsam/linear/PowerMethod.h>

namespace gtsam {

using Sparse = Eigen::SparseMatrix<double>;

/**
 * \brief Compute maximum Eigenpair with accelerated power method
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
 * It performs the following iteration: \f$ x_{k+1} = A * x_k - \beta *
 * x_{k-1} \f$ where A is the aim matrix we want to get eigenpair of, x is the
 * Ritz vector
 *
 * Template argument Operator just needs multiplication operator
 *
 */
template <class Operator>
class AcceleratedPowerMethod : public PowerMethod<Operator> {

  double beta_ = 0;  // a Polyak momentum term

  Vector previousVector_;  // store previous vector

 public:
  /**
   * Constructor from aim matrix A (given as Matrix or Sparse), optional intial
   * vector as ritzVector
   */
  explicit AcceleratedPowerMethod(
      const Operator &A, const std::optional<Vector> initial = {},
      double initialBeta = 0.0)
      : PowerMethod<Operator>(A, initial) {
    // initialize Ritz eigen vector and previous vector
    this->ritzVector_ = initial ? *initial : Vector::Random(this->dim_);
    this->ritzVector_.normalize();
    previousVector_ = Vector::Zero(this->dim_);

    // initialize beta_
    beta_ = initialBeta;
  }

  /**
   * Run accelerated power iteration to get ritzVector with beta and previous
   * two ritzVector x0 and x00, and return 
   *   \f$y = (A * x0 - \beta * x00) / || A * x0 - \beta * x00 ||\f$
   */
  Vector acceleratedPowerIteration (const Vector &x1, const Vector &x0,
                        const double beta) const {
    Vector y = this->A_ * x1 - beta * x0;
    y.normalize();
    return y;
  }

  /**
   * Run accelerated power iteration to get ritzVector with beta and previous
   * two ritzVector x0 and x00, and return
   *  \f$y = (A * x0 - \beta * x00) / || A * x0 - \beta * x00 ||\f$
   */
  Vector acceleratedPowerIteration () const {
    Vector y = acceleratedPowerIteration(this->ritzVector_, previousVector_, beta_);
    return y;
  }

  /**
   * Tuning the momentum beta using the Best Heavy Ball algorithm in Ref(3), T
   * is the iteration time to find beta with largest Rayleigh quotient
   */
  double estimateBeta(const size_t T = 10) const {
    // set initial estimation of maxBeta
    Vector initVector = this->ritzVector_;
    const double up = initVector.dot( this->A_ * initVector );
    const double down = initVector.dot(initVector);
    const double mu = up / down;
    double maxBeta = mu * mu / 4;
    size_t maxIndex;
    std::vector<double> betas;

    Matrix R = Matrix::Zero(this->dim_, 10);
    // run T times of iteration to find the beta that has the largest Rayleigh quotient
    for (size_t t = 0; t < T; t++) {
      // after each t iteration, reset the betas with the current maxBeta
      betas = {2 / 3 * maxBeta, 0.99 * maxBeta, maxBeta, 1.01 * maxBeta,
               1.5 * maxBeta};
      // iterate through every beta value
      for (size_t k = 0; k < betas.size(); ++k) {
        // initialize x0 and x00 in each iteration of each beta
        Vector x0 = initVector;
        Vector x00 = Vector::Zero(this->dim_);
        // run 10 steps of accelerated power iteration with this beta
        for (size_t j = 1; j < 10; j++) {
          if (j < 2) {
            R.col(0) = acceleratedPowerIteration(x0, x00, betas[k]);
            R.col(1) = acceleratedPowerIteration(R.col(0), x0, betas[k]);
          } else {
            R.col(j) = acceleratedPowerIteration(R.col(j - 1), R.col(j - 2),
  betas[k]);
          }
        }
        // compute the Rayleigh quotient for the randomly sampled vector after
        // 10 steps of power accelerated iteration
        const Vector x = R.col(9);
        const double up = x.dot(this->A_ * x);
        const double down = x.dot(x);
        const double mu = up / down;
        // store the momentum with largest Rayleigh quotient and its according index of beta_
        if (mu * mu / 4 > maxBeta) {
          // save the max beta index
          maxIndex = k;
          maxBeta = mu * mu / 4;
        }
      }
    }
    // set beta_ to momentum with largest Rayleigh quotient
    return betas[maxIndex];
  }

  /**
   * Start the accelerated iteration, after performing the
   * accelerated iteration, calculate the ritz error, repeat this
   * operation until the ritz error converge. If converged return true, else
   * false.
   */
  bool compute(size_t maxIterations, double tol) {
    // Starting
    bool isConverged = false;

    for (size_t i = 0; i < maxIterations && !isConverged; i++) {
      ++(this->nrIterations_);
      Vector tmp = this->ritzVector_;
      // update the ritzVector after accelerated power iteration
      this->ritzVector_ = acceleratedPowerIteration();
      // update the previousVector with ritzVector
      previousVector_ = tmp;
      // update the ritzValue 
      this->ritzValue_ = this->ritzVector_.dot(this->A_ * this->ritzVector_);
      isConverged = this->converged(tol);
    }

    return isConverged;
  }
};

}  // namespace gtsam
