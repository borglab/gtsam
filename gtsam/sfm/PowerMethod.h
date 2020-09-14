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

namespace gtsam {

using Sparse = Eigen::SparseMatrix<double>;

  /* ************************************************************************* */
/// MINIMUM EIGENVALUE COMPUTATIONS

/** This is a lightweight struct used in conjunction with Spectra to compute
 * the minimum eigenvalue and eigenvector of a sparse matrix A; it has a single
 * nontrivial function, perform_op(x,y), that computes and returns the product
 * y = (A + sigma*I) x */
struct PowerFunctor {
  // Const reference to an externally-held matrix whose minimum-eigenvalue we
  // want to compute
  const Sparse &A_;

  const int m_n_; // dimension of Matrix A
  const int m_nev_; // number of eigenvalues required

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
  explicit PowerFunctor(const Sparse &A, int nev, int ncv, 
                             double beta = 0)
      : A_(A),
        m_n_(A.rows()),
        m_nev_(nev),
        // m_ncv_(ncv > m_n_ ? m_n_ : ncv),
        beta_(beta),
        m_niter_(0)
         {}

  int rows() const { return A_.rows(); }
  int cols() const { return A_.cols(); }

  // Matrix-vector multiplication operation
  Vector perform_op(const Vector& x1, const Vector& x0) const {

    // Do the multiplication using wrapped Eigen vectors
    Vector x2 = A_ * x1 - beta_ * x0;
    x2.normalize();
    return x2;
  }

  Vector perform_op(int i) const {

    // Do the multiplication using wrapped Eigen vectors
    Vector x1 = ritz_vectors_.col(i-1);
    Vector x2 = ritz_vectors_.col(i-2);
    return perform_op(x1, x2);
  }

  long next_long_rand(long seed) {
    const unsigned int m_a = 16807;
    const unsigned long m_max = 2147483647L;
    // long m_rand = m_n_ ? (m_n_ & m_max) : 1;
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
    ritz_vectors_.col(0) = perform_op(x0, x00);
    ritz_vectors_.col(1) = perform_op(ritz_vectors_.col(0), x0);

    // setting beta
    Vector init_resid = ritz_vectors_.col(0);
    const double up = init_resid.transpose() * A_ * init_resid;
    const double down = init_resid.transpose().dot(init_resid);
    const double mu =  up/down;
    beta_ = mu*mu/4;

  }

  void init() {
    Vector x0 = random_vec(m_n_);
    Vector x00 = random_vec(m_n_);
    init(x0, x00);
  }

  bool converged(double tol, int i) {
    Vector x = ritz_vectors_.col(i);
    double theta = x.transpose() * A_ * x;

    // store the ritz eigen value
    ritz_val_(i) = theta;

    // update beta
    beta_ = std::max(beta_, theta * theta / 4);

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
      if (i>0 && i<ritz_vectors_.cols()-1) {
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

  int compute(int maxit, double tol) {
    // Starting
    int i = 0;
    int nconv = 0;
    for (; i < maxit; i++) {
      m_niter_ +=1;
      nconv = num_converged(tol);
      if (nconv >= m_nev_) break;
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
