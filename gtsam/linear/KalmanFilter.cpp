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
#include <gtsam/linear/GaussianBayesNet.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/base/Testable.h>

#include <boost/make_shared.hpp>

using namespace std;

namespace gtsam {

/* ************************************************************************* */
// Auxiliary function to solve factor graph and return pointer to root conditional
KalmanFilter::State //
KalmanFilter::solve(const GaussianFactorGraph& factorGraph) const {

  // Eliminate the graph using the provided Eliminate function
  Ordering ordering(factorGraph.keys());
  GaussianBayesNet::shared_ptr bayesNet = //
      factorGraph.eliminateSequential(ordering, function_);

  // As this is a filter, all we need is the posterior P(x_t).
  // This is the last GaussianConditional in the resulting BayesNet
  GaussianConditional::shared_ptr posterior = *(--bayesNet->end());
  return boost::make_shared<GaussianDensity>(*posterior);
}

/* ************************************************************************* */
// Auxiliary function to create a small graph for predict or update and solve
KalmanFilter::State //
KalmanFilter::fuse(const State& p, GaussianFactor::shared_ptr newFactor) const {

  // Create a factor graph
  GaussianFactorGraph factorGraph;
  factorGraph += p, newFactor;

  // Eliminate graph in order x0, x1, to get Bayes net P(x0|x1)P(x1)
  return solve(factorGraph);
}

/* ************************************************************************* */
KalmanFilter::State KalmanFilter::init(const Vector& x0,
    const SharedDiagonal& P0) const {

  // Create a factor graph f(x0), eliminate it into P(x0)
  GaussianFactorGraph factorGraph;
  factorGraph += JacobianFactor(0, I_, x0, P0); // |x-x0|^2_diagSigma
  return solve(factorGraph);
}

/* ************************************************************************* */
KalmanFilter::State KalmanFilter::init(const Vector& x, const Matrix& P0) const {

  // Create a factor graph f(x0), eliminate it into P(x0)
  GaussianFactorGraph factorGraph;
  factorGraph += HessianFactor(0, x, P0); // 0.5*(x-x0)'*inv(Sigma)*(x-x0)
  return solve(factorGraph);
}

/* ************************************************************************* */
void KalmanFilter::print(const string& s) const {
  cout << "KalmanFilter " << s << ", dim = " << n_ << endl;
}

/* ************************************************************************* */
KalmanFilter::State KalmanFilter::predict(const State& p, const Matrix& F,
    const Matrix& B, const Vector& u, const SharedDiagonal& model) const {

  // The factor related to the motion model is defined as
  // f2(x_{t},x_{t+1}) = (F*x_{t} + B*u - x_{t+1}) * Q^-1 * (F*x_{t} + B*u - x_{t+1})^T
  Key k = step(p);
  return fuse(p,
      boost::make_shared<JacobianFactor>(k, -F, k + 1, I_, B * u, model));
}

/* ************************************************************************* */
KalmanFilter::State KalmanFilter::predictQ(const State& p, const Matrix& F,
    const Matrix& B, const Vector& u, const Matrix& Q) const {

#ifndef NDEBUG
  DenseIndex n = F.cols();
  assert(F.rows() == n);
  assert(B.rows() == n);
  assert(B.cols() == u.size());
  assert(Q.rows() == n);
  assert(Q.cols() == n);
#endif

  // The factor related to the motion model is defined as
  // f2(x_{t},x_{t+1}) = (F*x_{t} + B*u - x_{t+1}) * Q^-1 * (F*x_{t} + B*u - x_{t+1})^T
  // See documentation in HessianFactor, we have A1 = -F,  A2 = I_, b = B*u:
  // TODO: starts to seem more elaborate than straight-up KF equations?
  Matrix M = Q.inverse(), Ft = trans(F);
  Matrix G12 = -Ft * M, G11 = -G12 * F, G22 = M;
  Vector b = B * u, g2 = M * b, g1 = -Ft * g2;
  double f = dot(b, g2);
  Key k = step(p);
  return fuse(p,
      boost::make_shared<HessianFactor>(k, k + 1, G11, G12, g1, G22, g2, f));
}

/* ************************************************************************* */
KalmanFilter::State KalmanFilter::predict2(const State& p, const Matrix& A0,
    const Matrix& A1, const Vector& b, const SharedDiagonal& model) const {
  // Nhe factor related to the motion model is defined as
  // f2(x_{t},x_{t+1}) = |A0*x_{t} + A1*x_{t+1} - b|^2
  Key k = step(p);
  return fuse(p, boost::make_shared<JacobianFactor>(k, A0, k + 1, A1, b, model));
}

/* ************************************************************************* */
KalmanFilter::State KalmanFilter::update(const State& p, const Matrix& H,
    const Vector& z, const SharedDiagonal& model) const {
  // The factor related to the measurements would be defined as
  // f2 = (h(x_{t}) - z_{t}) * R^-1 * (h(x_{t}) - z_{t})^T
  //    = (x_{t} - z_{t}) * R^-1 * (x_{t} - z_{t})^T
  Key k = step(p);
  return fuse(p, boost::make_shared<JacobianFactor>(k, H, z, model));
}

/* ************************************************************************* */
KalmanFilter::State KalmanFilter::updateQ(const State& p, const Matrix& H,
    const Vector& z, const Matrix& Q) const {
  Key k = step(p);
  Matrix M = Q.inverse(), Ht = trans(H);
  Matrix G = Ht * M * H;
  Vector g = Ht * M * z;
  double f = dot(z, M * z);
  return fuse(p, boost::make_shared<HessianFactor>(k, G, g, f));
}

/* ************************************************************************* */

} // \namespace gtsam

