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
 * @brief  accelerated power method for fast eigenvalue and eigenvector computation
 */

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/dllexport.h>

#include <Eigen/Core>
#include <Eigen/Sparse>
#include <map>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>
#include <algorithm>
#include <cmath>
#include <chrono>
#include <random>

namespace gtsam {

using Sparse = Eigen::SparseMatrix<double>;

/* ************************************************************************* */
/// MINIMUM EIGENVALUE COMPUTATIONS

struct PowerFunctor {
  /**
   * \brief Computer i-th Eigenpair with power method
   *
   * References :
   * 1) Rosen, D. and Carlone, L., 2017, September. Computational
   * enhancements for certifiably correct SLAM. In Proceedings of the
   * International Conference on Intelligent Robots and Systems.
   * 2) Yulun Tian and Kasra Khosoussi and David M. Rosen and Jonathan P. How,
   * 2020, Aug, Distributed Certifiably Correct Pose-Graph Optimization, Arxiv
   * 3) C. de Sa, B. He, I. Mitliagkas, C. Ré, and P. Xu, “Accelerated
   * stochastic power iteration,” in Proc. Mach. Learn. Res., no. 84, 2018, pp. 58–67
   *
   * It performs the following iteration: \f$ x_{k+1} = A * x_k + \beta *
   * x_{k-1} \f$ where A is the certificate matrix, x is the ritz vector
   *
   */

  // Const reference to an externally-held matrix whose minimum-eigenvalue we
  // want to compute
  const Sparse &A_;

  const Matrix &S_;

  const int m_n_; // dimension of Matrix A
  const int m_nev_; // number of eigenvalues required

  // flag for running power method or accelerated power method. If false, the former, vice versa.
  bool accelerated_;

  // a Polyak momentum term
  double beta_;

  // const int m_ncv_; // dimention of orthonormal basis subspace
  size_t m_niter_; // number of iterations

private:
  Vector ritz_val_; // all ritz eigenvalues
  Matrix ritz_vectors_; // all ritz eigenvectors
  Vector ritz_conv_; // store whether the ritz eigenpair converged
  Vector sorted_ritz_val_; // sorted converged eigenvalue
  Matrix sorted_ritz_vectors_; // sorted converged eigenvectors

public:
 // Constructor
 explicit PowerFunctor(const Sparse& A, const Matrix& S, int nev, int ncv,
                       bool accelerated = false, double beta = 0)
     : A_(A),
       S_(S),
       m_n_(A.rows()),
       m_nev_(nev),
       // m_ncv_(ncv > m_n_ ? m_n_ : ncv),
       accelerated_(accelerated),
       beta_(beta),
       m_niter_(0) {
         // Do nothing
       }

 int rows() const { return A_.rows(); }
 int cols() const { return A_.cols(); }

 // Matrix-vector multiplication operation
 Vector perform_op(const Vector& x1, const Vector& x0) const {
   // Do the multiplication
   Vector x2 = A_ * x1 - beta_ * x0;
   x2.normalize();
   return x2;
  }

  Vector perform_op(const Vector& x1, const Vector& x0, const double beta) const {
    Vector x2 = A_ * x1 - beta * x0;
    x2.normalize();
    return x2;
  }

  Vector perform_op(const Vector& x1) const {
    Vector x2 = A_ * x1;
    x2.normalize();
    return x2;
  }

  Vector perform_op(int i) const {
    if (accelerated_) {
      Vector x1 = ritz_vectors_.col(i-1);
      Vector x2 = ritz_vectors_.col(i-2);
      return perform_op(x1, x2);
    } else
      return perform_op(ritz_vectors_.col(i-1));
  }

  long next_long_rand(long seed) {
    const unsigned int m_a = 16807;
    const unsigned long m_max = 2147483647L;
    unsigned long lo, hi;

    lo = m_a * (long)(seed & 0xFFFF);
    hi = m_a * (long)((unsigned long)seed >> 16);
    lo += (hi & 0x7FFF) << 16;
    if (lo > m_max) {
      lo &= m_max;
      ++lo;
    }
    lo += hi >> 15;
    if (lo > m_max) {
      lo &= m_max;
      ++lo;
    }
    return (long)lo;
  }

  Vector random_vec(const int len) {
    Vector res(len);
    const unsigned long m_max = 2147483647L;
    for (int i = 0; i < len; i++) {
      long m_rand = next_long_rand(m_rand);
      res[i] = double(m_rand) / double(m_max) - double(0.5);
    }
    res.normalize();
    return res;
  }

  /// Tuning the momentum beta using the Best Heavy Ball algorithm in Ref(3)
  void setBeta() {
    if (m_n_ < 10) return;
    double maxBeta = beta_;
    size_t maxIndex;
    std::vector<double> betas = {2/3*maxBeta, 0.99*maxBeta, maxBeta, 1.01*maxBeta, 1.5*maxBeta};

    Matrix tmp_ritz_vectors;
    tmp_ritz_vectors.resize(m_n_, 10);
    tmp_ritz_vectors.setZero();
    for (size_t i = 0; i < 10; i++) {
      for (size_t k = 0; k < betas.size(); ++k) {
        for (size_t j = 1; j < 10; j++) {
          // double rayleighQuotient;
          if (j <2 ) {
            Vector x0 = random_vec(m_n_);
            Vector x00 = random_vec(m_n_);
            tmp_ritz_vectors.col(0) = perform_op(x0, x00, betas[k]);
            tmp_ritz_vectors.col(1) =
                perform_op(tmp_ritz_vectors.col(0), x0, betas[k]);
          }
          else {
            tmp_ritz_vectors.col(j) =
                perform_op(tmp_ritz_vectors.col(j - 1),
                           tmp_ritz_vectors.col(j - 2), betas[k]);
          }
          const Vector x = tmp_ritz_vectors.col(j);
          const double up = x.transpose() * A_ * x;
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

  void perturb(int i) {
    // generate a 0.03*||x_0||_2 as stated in David's paper
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::mt19937 generator (seed);
    std::uniform_real_distribution<double> uniform01(0.0, 1.0);

    int n = m_n_;
    Vector disturb;
    disturb.resize(n);
    disturb.setZero();
    for (int i =0; i<n; ++i) {
      disturb(i) = uniform01(generator);
    }
    disturb.normalize();

    Vector x0 = ritz_vectors_.col(i);
    double magnitude = x0.norm();
    ritz_vectors_.col(i) = x0 + 0.03 * magnitude * disturb;
  }

  void init(const Vector x0, const Vector x00) {
    // initialzie ritz eigen values
    ritz_val_.resize(m_n_);
    ritz_val_.setZero();

    // initialzie the ritz converged vector
    ritz_conv_.resize(m_n_);
    ritz_conv_.setZero();

    // initialzie ritz eigen vectors
    ritz_vectors_.resize(m_n_, m_n_);
    ritz_vectors_.setZero();
    if (accelerated_){
      ritz_vectors_.col(0) = perform_op(x0, x00);
      ritz_vectors_.col(1) = perform_op(ritz_vectors_.col(0), x0);
    } else {
      ritz_vectors_.col(0) = perform_op(x0);
      perturb(0);
    }

    // setting beta
    if (accelerated_) {
      Vector init_resid = ritz_vectors_.col(0);
      const double up = init_resid.transpose() * A_ * init_resid;
      const double down = init_resid.transpose().dot(init_resid);
      const double mu =  up/down;
      beta_ = mu*mu/4;
      setBeta();
    }
    
  }

  void init() {
    Vector x0;
    Vector x00;
    if (!S_.isZero(0)) {
      x0 = S_.row(1);
      x00 = S_.row(0);
    } else {
      x0 = random_vec(m_n_);
      x00 = random_vec(m_n_);
    }
    init(x0, x00);
  }

  bool converged(double tol, int i) {
    Vector x = ritz_vectors_.col(i);
    double theta = x.transpose() * A_ * x;

    // store the ritz eigen value
    ritz_val_(i) = theta;

    // update beta
    if (accelerated_) beta_ = std::max(beta_, theta * theta / 4);

    Vector diff = A_ * x - theta * x;
    double error = diff.lpNorm<1>();
    if (error < tol) ritz_conv_(i) = 1;
    return error < tol;
  }

  int num_converged(double tol) {
    int num_converge = 0;
    for (int i=0; i<ritz_vectors_.cols(); i++) {
      if (converged(tol, i)) {
        num_converge += 1;
      }
      if (!accelerated_ && i<ritz_vectors_.cols()-1) {
        ritz_vectors_.col(i+1) = perform_op(i+1);
      } else if (accelerated_ && i>0 && i<ritz_vectors_.cols()-1) {
        ritz_vectors_.col(i+1) = perform_op(i+1);
      }
    }
    return num_converge;
  }

  size_t num_iterations() {
    return m_niter_;
  }

  void sort_eigenpair() {
    std::vector<std::pair<double, int>> pairs;
    for(int i=0; i<ritz_conv_.size(); ++i) {
      if (ritz_conv_(i)) pairs.push_back({ritz_val_(i), i});
    }

    std::sort(pairs.begin(), pairs.end(), [](const std::pair<double, int>& left, const std::pair<double, int>& right) {
      return left.first < right.first;
    });

    // initialzie sorted ritz eigenvalues and eigenvectors
    size_t num_converged = pairs.size();
    sorted_ritz_val_.resize(num_converged);
    sorted_ritz_val_.setZero();
    sorted_ritz_vectors_.resize(m_n_, num_converged);
    sorted_ritz_vectors_.setZero();
    
    // fill sorted ritz eigenvalues and eigenvectors with sorted index
    for(size_t j=0; j<num_converged; ++j) {
      sorted_ritz_val_(j) = pairs[j].first;
      sorted_ritz_vectors_.col(j) = ritz_vectors_.col(pairs[j].second);
    }
  }

  void reset() {
    if (accelerated_) {
      ritz_vectors_.col(0) = perform_op(ritz_vectors_.col(m_n_-1-1), ritz_vectors_.col(m_n_-1-2));
      ritz_vectors_.col(1) = perform_op(ritz_vectors_.col(0), ritz_vectors_.col(m_n_-1-1));
    } else {
      ritz_vectors_.col(0) = perform_op(ritz_vectors_.col(m_n_-1));
    }
  }

  int compute(int maxit, double tol) {
    // Starting
    int i = 0;
    int nconv = 0;
    for (; i < maxit; i++) {
      m_niter_ +=1;
      nconv = num_converged(tol);
      if (nconv >= m_nev_) break;
      else reset();
    }

    // sort the result
    sort_eigenpair();

    return std::min(m_nev_, nconv);
  }

  Vector eigenvalues() {
    return sorted_ritz_val_;
  }

  Matrix eigenvectors() {
    return sorted_ritz_vectors_;
  }

};

} // namespace gtsam
