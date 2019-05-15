/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file KalmanFilter.h
 * @brief Simple linear Kalman filter. Implemented using factor graphs, i.e., does Cholesky-based SRIF, really.
 * @date Sep 3, 2011
 * @author Stephen Williams
 * @author Frank Dellaert
 */

#pragma once

#include <gtsam/linear/GaussianDensity.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>

#ifndef KALMANFILTER_DEFAULT_FACTORIZATION
#define KALMANFILTER_DEFAULT_FACTORIZATION QR
#endif

namespace gtsam {

/**
 * Kalman Filter class
 *
 * Knows how to maintain a Gaussian density under linear-Gaussian motion and
 * measurement models. It uses the square-root information form, as usual in GTSAM.
 *
 * The filter is functional, in that it does not have state: you call init() to create
 * an initial state, then predict() and update() that create new states out of an old state.
 */
class GTSAM_EXPORT KalmanFilter {

public:

  /**
   *  This Kalman filter is a Square-root Information filter
   *  The type below allows you to specify the factorization variant.
   */
  enum Factorization {
    QR, CHOLESKY
  };

  /**
   * The Kalman filter state is simply a GaussianDensity
   */
  typedef GaussianDensity::shared_ptr State;

private:

  const size_t n_; /** dimensionality of state */
  const Matrix I_; /** identity matrix of size n*n */
  const GaussianFactorGraph::Eliminate function_; /** algorithm */

  State solve(const GaussianFactorGraph& factorGraph) const;
  State fuse(const State& p, GaussianFactor::shared_ptr newFactor) const;

public:

  // Constructor
  KalmanFilter(size_t n, Factorization method =
      KALMANFILTER_DEFAULT_FACTORIZATION) :
      n_(n), I_(Matrix::Identity(n_, n_)), function_(
          method == QR ? GaussianFactorGraph::Eliminate(EliminateQR) :
              GaussianFactorGraph::Eliminate(EliminateCholesky)) {
  }

  /**
   * Create initial state, i.e., prior density at time k=0
   * In Kalman Filter notation, these are x_{0|0} and P_{0|0}
   * @param x0 estimate at time 0
   * @param P0 covariance at time 0, given as a diagonal Gaussian 'model'
   */
  State init(const Vector& x0, const SharedDiagonal& P0) const;

  /// version of init with a full covariance matrix
  State init(const Vector& x0, const Matrix& P0) const;

  /// print
  void print(const std::string& s = "") const;

  /** Return step index k, starts at 0, incremented at each predict. */
  static Key step(const State& p) {
    return p->firstFrontalKey();
  }

  /**
   * Predict the state P(x_{t+1}|Z^t)
   *   In Kalman Filter notation, this is x_{t+1|t} and P_{t+1|t}
   * Details and parameters:
   *   In a linear Kalman Filter, the motion model is f(x_{t}) = F*x_{t} + B*u_{t} + w
   *   where F is the state transition model/matrix, B is the control input model,
   *   and w is zero-mean, Gaussian white noise with covariance Q.
   */
  State predict(const State& p, const Matrix& F, const Matrix& B,
      const Vector& u, const SharedDiagonal& modelQ) const;

  /*
   *  Version of predict with full covariance
   *  Q is normally derived as G*w*G^T where w models uncertainty of some
   *  physical property, such as velocity or acceleration, and G is derived from physics.
   *  This version allows more realistic models than a diagonal covariance matrix.
   */
  State predictQ(const State& p, const Matrix& F, const Matrix& B,
      const Vector& u, const Matrix& Q) const;

  /**
   * Predict the state P(x_{t+1}|Z^t)
   *   In Kalman Filter notation, this is x_{t+1|t} and P_{t+1|t}
   *   After the call, that is the density that can be queried.
   * Details and parameters:
   *   This version of predict takes GaussianFactor motion model [A0 A1 b]
   *   with an optional noise model.
   */
  State predict2(const State& p, const Matrix& A0, const Matrix& A1,
      const Vector& b, const SharedDiagonal& model) const;

  /**
   * Update Kalman filter with a measurement
   * For the Kalman Filter, the measurement function, h(x_{t}) = z_{t}
   * will be of the form h(x_{t}) = H*x_{t} + v
   * where H is the observation model/matrix, and v is zero-mean,
   * Gaussian white noise with covariance R.
   * In this version, R is restricted to diagonal Gaussians (model parameter)
   */
  State update(const State& p, const Matrix& H, const Vector& z,
      const SharedDiagonal& model) const;

  /*
   *  Version of update with full covariance
   *  Q is normally derived as G*w*G^T where w models uncertainty of some
   *  physical property, such as velocity or acceleration, and G is derived from physics.
   *  This version allows more realistic models than a diagonal covariance matrix.
   */
  State updateQ(const State& p, const Matrix& H, const Vector& z,
      const Matrix& Q) const;
};

} // \namespace gtsam

/* ************************************************************************* */

