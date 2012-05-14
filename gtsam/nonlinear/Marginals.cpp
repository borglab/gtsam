/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Marginals.cpp
 * @brief 
 * @author Richard Roberts
 * @date May 14, 2012
 */

#include <gtsam/3rdparty/Eigen/Eigen/Dense>
#include <gtsam/linear/GaussianMultifrontalSolver.h>
#include <gtsam/nonlinear/Marginals.h>

namespace gtsam {

Marginals::Marginals(const NonlinearFactorGraph& graph, const Values& solution, Factorization factorization) {

  // Compute COLAMD ordering
  ordering_ = *graph.orderingCOLAMD(solution);

  // Linearize graph
  graph_ = *graph.linearize(solution, ordering_);

  // Compute BayesTree
  factorization_ = factorization;
  if(factorization_ == CHOLESKY)
    bayesTree_ = *GaussianMultifrontalSolver(graph_, false).eliminate();
  else if(factorization_ == QR)
    bayesTree_ = *GaussianMultifrontalSolver(graph_, true).eliminate();
}

Matrix Marginals::marginalCovariance(Key variable) const {
  // Get linear key
  Index index = ordering_[variable];

  // Compute marginal
  GaussianFactor::shared_ptr marginalFactor;
  if(factorization_ == CHOLESKY)
    marginalFactor = bayesTree_.marginalFactor(index, EliminatePreferCholesky);
  else if(factorization_ == QR)
    marginalFactor = bayesTree_.marginalFactor(index, EliminateQR);

  // Get information matrix (only store upper-right triangle)
  Matrix info;
  if(typeid(*marginalFactor) == typeid(JacobianFactor)) {
    JacobianFactor::constABlock A = static_cast<const JacobianFactor&>(*marginalFactor).getA();
    info = A.transpose() * A; // Compute A'A
  } else if(typeid(*marginalFactor) == typeid(HessianFactor)) {
    const HessianFactor& hessian = static_cast<const HessianFactor&>(*marginalFactor);
    const size_t dim = hessian.getDim(hessian.begin());
    info = hessian.info().topLeftCorner(dim,dim).selfadjointView<Eigen::Upper>(); // Take the non-augmented part of the information matrix
  }

  // Compute covariance
  return info.inverse();
}

} /* namespace gtsam */
