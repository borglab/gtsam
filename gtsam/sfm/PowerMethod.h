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
#include <Eigen/Array>
#include <map>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

namespace gtsam {
  /* ************************************************************************* */
/// MINIMUM EIGENVALUE COMPUTATIONS

/** This is a lightweight struct used in conjunction with Spectra to compute
 * the minimum eigenvalue and eigenvector of a sparse matrix A; it has a single
 * nontrivial function, perform_op(x,y), that computes and returns the product
 * y = (A + sigma*I) x */
struct MatrixProdFunctor {
  // Const reference to an externally-held matrix whose minimum-eigenvalue we
  // want to compute
  const Sparse &A_;

  // Spectral shift
  double sigma_;

  const int m_n_;
  const int m_nev_;
  const int m_ncv_;
  Vector ritz_val_;
  Matrix ritz_vectors_;

  // Constructor
  explicit MatrixProdFunctor(const Sparse &A, int nev, int ncv,
                             double sigma = 0)
      : A_(A),
        m_n_(A.rows()),
        m_nev_(nev),
        m_ncv_(ncv > m_n_ ? m_n_ : ncv),
        sigma_(sigma) {}

  int rows() const { return A_.rows(); }
  int cols() const { return A_.cols(); }

  // Matrix-vector multiplication operation
  void perform_op(const double *x, double *y) const {
    // Wrap the raw arrays as Eigen Vector types
    Eigen::Map<const Vector> X(x, rows());
    Eigen::Map<Vector> Y(y, rows());

    // Do the multiplication using wrapped Eigen vectors
    Y = A_ * X + sigma_ * X;
  }

  long next_long_rand(long seed) {
    const unsigned int m_a = 16807;
    const unsigned long m_max = 2147483647L;
    long m_rand = m_n_ ? (m_n_ & m_max) : 1;
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
    return res;
  }

  void init(Vector data) {
    ritz_val_.resize(m_ncv_);
    ritz_val_.setZero();
    ritz_vectors_.resize(m_ncv_, m_nev_);
    ritz_vectors_.setZero();
    Eigen::Map<Matrix> v0(init_resid.data(), m_n_);
  }

  void init() {
    Vector init_resid = random_vec(m_n_);
    init(init_resid);
  }

  bool converged(double tol, const Vector x) {
    double theta = x.transpose() * A_ * x;
    Vector diff = A_ * x - theta * x;
    double error = diff.lpNorm<1>();
    return error < tol;
  }

  int num_converged(double tol) {
    int converge = 0;
    for (int i=0; i<ritz_vectors_.cols(); i++) {
      if (converged(tol, ritz_vectors_.col(i))) converged += 1;
    }
    return converged;
  }

  int compute(int maxit, double tol) {
    // Starting
    int i, nconv = 0, nev_adj;
    for (i = 0; i < maxit; i++) {
      nconv = num_converged(tol);
      if (nconv >= m_nev_) break;

      nev_adj = nev_adjusted(nconv);
      restart(nev_adj);
    }
    // Sorting results
    sort_ritzpair(sort_rule);

    m_niter += i + 1;
    m_info = (nconv >= m_nev_) ? SUCCESSFUL : NOT_CONVERGING;

    return std::min(m_nev_, nconv);
  }

  Vector eigenvalues() {
    return ritz_val_;
  }

  Matrix eigenvectors() {
    return ritz_vectors_;
  }

};

} // namespace gtsam
