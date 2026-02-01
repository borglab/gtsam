/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file KalmanFilter.cpp
 *
 * @brief Simple linear Kalman filter.
 * Implemented using factor graphs, i.e., does Cholesky-based SRIF, really.
 *
 * @date Sep 3, 2011
 * @author Stephen Williams
 * @author Frank Dellaert
 */


#include <gtsam/linear/KalmanFilter.h>

#include <gtsam/base/Testable.h>
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/JacobianFactor.h>

#ifndef NDEBUG
#include <cassert>
#endif

// In the code below we often get a covariance matrix Q, and we need to create a
// JacobianFactor with cost 0.5 * |Ax - b|^T Q^{-1} |Ax - b|. Factorizing Q as
//   Q = L L^T
// and hence
//   Q^{-1} = L^{-T} L^{-1} = M^T M, with M = L^{-1}
// We then have
//   0.5 * |Ax - b|^T Q^{-1} |Ax - b|
// = 0.5 * |Ax - b|^T M^T M |Ax - b|
// = 0.5 * |MAx - Mb|^T |MAx - Mb|
// The functor below efficiently multiplies with M by calling L.solve().
namespace {
using Matrix = gtsam::Matrix;
struct InverseL : public Eigen::LLT<Matrix> {
  InverseL(const Matrix& Q) : Eigen::LLT<Matrix>(Q) {}
  Matrix operator*(const Matrix& A) const { return matrixL().solve(A); }
};
}  // namespace

namespace gtsam {
/* ************************************************************************* */
// Auxiliary function to solve factor graph and return pointer to root
// conditional
KalmanFilter::State KalmanFilter::solve(
    const GaussianFactorGraph& factorGraph) const {
  // Eliminate the graph using the provided Eliminate function
  Ordering ordering(factorGraph.keys());
  const auto bayesNet = factorGraph.eliminateSequential(ordering, function_);

  // As this is a filter, all we need is the posterior P(x_t).
  // This is the last GaussianConditional in the resulting BayesNet
  GaussianConditional::shared_ptr posterior = bayesNet->back();
  return std::make_shared<GaussianDensity>(*posterior);
}

/* ************************************************************************* */
// Auxiliary function to create a small graph for predict or update and solve
KalmanFilter::State KalmanFilter::fuse(
    const State& p, GaussianFactor::shared_ptr newFactor) const {
  // Create a factor graph
  GaussianFactorGraph factorGraph;
  factorGraph.push_back(p);
  factorGraph.push_back(newFactor);

  // Eliminate graph in order x0, x1, to get Bayes net P(x0|x1)P(x1)
  return solve(factorGraph);
}

/* ************************************************************************* */
KalmanFilter::State KalmanFilter::init(const Vector& x0,
                                       const SharedDiagonal& P0) const {
  // Create a factor graph f(x0), eliminate it into P(x0)
  GaussianFactorGraph factorGraph;
  factorGraph.emplace_shared<JacobianFactor>(0, I_, x0,
                                             P0);  // |x-x0|^2_P0
  return solve(factorGraph);
}

/* ************************************************************************* */
KalmanFilter::State KalmanFilter::init(const Vector& x0,
                                       const Matrix& P0) const {
  // Create a factor graph f(x0), eliminate it into P(x0)
  GaussianFactorGraph factorGraph;

  // Perform Cholesky decomposition of P0 = LL^T
  InverseL M(P0);

  // Premultiply I and x0 with M=L^{-1}
  const Matrix A = M * I_;  // = M
  const Vector b = M * x0;  // = M*x0
  factorGraph.emplace_shared<JacobianFactor>(0, A, b);
  return solve(factorGraph);
}

/* ************************************************************************* */
void KalmanFilter::print(const std::string& s) const {
  std::cout << "KalmanFilter " << s << ", dim = " << n_ << std::endl;
}

/* ************************************************************************* */
KalmanFilter::State KalmanFilter::predict(const State& p, const Matrix& F,
                                          const Matrix& B, const Vector& u,
                                          const SharedDiagonal& model) const {
  // The factor related to the motion model is defined as
  // f2(x_{k},x_{k+1}) =
  // (F*x_{k} + B*u - x_{k+1}) * Q^-1 * (F*x_{k} + B*u - x_{k+1})^T
  Key k = step(p);
  return fuse(p,
              std::make_shared<JacobianFactor>(k, -F, k + 1, I_, B * u, model));
}

/* ************************************************************************* */
KalmanFilter::State KalmanFilter::predictQ(const State& p, const Matrix& F,
                                           const Matrix& B, const Vector& u,
                                           const Matrix& Q) const {
#ifndef NDEBUG
  const DenseIndex n = dim();
  assert(F.cols() == n);
  assert(F.rows() == n);
  assert(B.rows() == n);
  assert(B.cols() == u.size());
  assert(Q.rows() == n);
  assert(Q.cols() == n);
#endif

  // Perform Cholesky decomposition of Q = LL^T
  InverseL M(Q);

  // Premultiply -F, I, and B * u with M=L^{-1}
  const Matrix A1 = -(M * F);
  const Matrix A2 = M * I_;
  const Vector b = M * (B * u);
  return predict2(p, A1, A2, b);
}

/* ************************************************************************* */
KalmanFilter::State KalmanFilter::predict2(const State& p, const Matrix& A0,
                                           const Matrix& A1, const Vector& b,
                                           const SharedDiagonal& model) const {
  // Nhe factor related to the motion model is defined as
  // f2(x_{k},x_{k+1}) = |A0*x_{k} + A1*x_{k+1} - b|^2
  Key k = step(p);
  return fuse(p, std::make_shared<JacobianFactor>(k, A0, k + 1, A1, b, model));
}

/* ************************************************************************* */
KalmanFilter::State KalmanFilter::update(const State& p, const Matrix& H,
                                         const Vector& z,
                                         const SharedDiagonal& model) const {
  // The factor related to the measurements would be defined as
  // f2 = (h(x_{k}) - z_{k}) * R^-1 * (h(x_{k}) - z_{k})^T
  //    = (x_{k} - z_{k}) * R^-1 * (x_{k} - z_{k})^T
  Key k = step(p);
  return fuse(p, std::make_shared<JacobianFactor>(k, H, z, model));
}

/* ************************************************************************* */
KalmanFilter::State KalmanFilter::updateQ(const State& p, const Matrix& H,
                                          const Vector& z,
                                          const Matrix& Q) const {
  Key k = step(p);

  // Perform Cholesky decomposition of Q = LL^T
  InverseL M(Q);

  // Pre-multiply H and z with M=L^{-1}, respectively
  const Matrix A = M * H;
  const Vector b = M * z;
  return fuse(p, std::make_shared<JacobianFactor>(k, A, b));
}

/* ************************************************************************* */

}  // namespace gtsam
