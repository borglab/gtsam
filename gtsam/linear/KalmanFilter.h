/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file KalmanFilter.h
 * @brief Simple linear Kalman filter implemented using factor graphs, i.e.,
 * performs Cholesky or QR-based SRIF (Square-Root Information Filter).
 * @date Sep 3, 2011
 * @authors Stephen Williams, Frank Dellaert
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
 * Maintains a Gaussian density under linear-Gaussian motion and
 * measurement models using the square-root information form.
 *
 * The filter is functional; it does not maintain internal state. Instead:
 * - Use `init()` to create an initial filter state,
 * - Call `predict()` and `update()` to create new states.
 */
class GTSAM_EXPORT KalmanFilter {
 public:
  /**
   * @enum Factorization
   * @brief Specifies the factorization variant to use.
   */
  enum Factorization { QR, CHOLESKY };

  /**
   * @typedef State
   * @brief The Kalman filter state, represented as a shared pointer to a
   * GaussianDensity.
   */
  typedef GaussianDensity::shared_ptr State;

 private:
  const size_t n_;  ///< Dimensionality of the state.
  const Matrix I_;  ///< Identity matrix of size \f$ n \times n \f$.
  const GaussianFactorGraph::Eliminate
      function_;  ///< Elimination algorithm used.

  /**
   * Solve the factor graph.
   * @param factorGraph The Gaussian factor graph to solve.
   * @return The resulting Kalman filter state.
   */
  State solve(const GaussianFactorGraph& factorGraph) const;

  /**
   * Fuse two states.
   * @param p The prior state.
   * @param newFactor The new factor to incorporate.
   * @return The resulting fused state.
   */
  State fuse(const State& p, GaussianFactor::shared_ptr newFactor) const;

 public:
  /**
   * Constructor.
   * @param n Dimensionality of the state.
   * @param method Factorization method (default: QR unless compile-flag set).
   */
  KalmanFilter(size_t n,
               Factorization method = KALMANFILTER_DEFAULT_FACTORIZATION)
      : n_(n),
        I_(Matrix::Identity(n_, n_)),
        function_(method == QR
                      ? GaussianFactorGraph::Eliminate(EliminateQR)
                      : GaussianFactorGraph::Eliminate(EliminateCholesky)) {}

  /**
   * Create the initial state (prior density at time \f$ k=0 \f$).
   *
   * In Kalman Filter notation:
   * - \f$ x_{0|0} \f$: Initial state estimate.
   * - \f$ P_{0|0} \f$: Initial covariance matrix.
   *
   * @param x0 Estimate of the state at time 0 (\f$ x_{0|0} \f$).
   * @param P0 Covariance matrix (\f$ P_{0|0} \f$), given as a diagonal Gaussian
   * model.
   * @return Initial Kalman filter state.
   */
  State init(const Vector& x0, const SharedDiagonal& P0) const;

  /**
   * Create the initial state with a full covariance matrix.
   * @param x0 Initial state estimate.
   * @param P0 Full covariance matrix.
   * @return Initial Kalman filter state.
   */
  State init(const Vector& x0, const Matrix& P0) const;

  /**
   * Print the Kalman filter details.
   * @param s Optional string prefix.
   */
  void print(const std::string& s = "") const;

  /**
   * Return the step index \f$ k \f$ (starts at 0, incremented at each predict
   * step).
   * @param p The current state.
   * @return Step index.
   */
  static Key step(const State& p) { return p->firstFrontalKey(); }

  /**
   * Predict the next state \f$ P(x_{k+1}|Z^k) \f$.
   *
   * In Kalman Filter notation:
   * - \f$ x_{k+1|k} \f$: Predicted state.
   * - \f$ P_{k+1|k} \f$: Predicted covariance.
   *
   * Motion model:
   * \f[
   * x_{k+1} = F \cdot x_k + B \cdot u_k + w
   * \f]
   * where \f$ w \f$ is zero-mean Gaussian noise with covariance \f$ Q \f$.
   *
   * @param p Previous state (\f$ x_k \f$).
   * @param F State transition matrix (\f$ F \f$).
   * @param B Control input matrix (\f$ B \f$).
   * @param u Control vector (\f$ u_k \f$).
   * @param modelQ Noise model (\f$ Q \f$, diagonal Gaussian).
   * @return Predicted state (\f$ x_{k+1|k} \f$).
   */
  State predict(const State& p, const Matrix& F, const Matrix& B,
                const Vector& u, const SharedDiagonal& modelQ) const;

  /**
   * Predict the next state with a full covariance matrix.
   *
   *@note Q is normally derived as G*w*G^T where w models uncertainty of some
   * physical property, such as velocity or acceleration, and G is derived from
   * physics. This version allows more realistic models than a diagonal matrix.
   *
   * @param p Previous state.
   * @param F State transition matrix.
   * @param B Control input matrix.
   * @param u Control vector.
   * @param Q Full covariance matrix (\f$ Q \f$).
   * @return Predicted state.
   */
  State predictQ(const State& p, const Matrix& F, const Matrix& B,
                 const Vector& u, const Matrix& Q) const;

  /**
   * Predict the next state using a GaussianFactor motion model.
   * @param p Previous state.
   * @param A0 Factor matrix.
   * @param A1 Factor matrix.
   * @param b Constant term vector.
   * @param model Noise model (optional).
   * @return Predicted state.
   */
  State predict2(const State& p, const Matrix& A0, const Matrix& A1,
                 const Vector& b, const SharedDiagonal& model = nullptr) const;

  /**
   * Update the Kalman filter with a measurement.
   *
   * Observation model:
   * \f[
   * z_k = H \cdot x_k + v
   * \f]
   * where \f$ v \f$ is zero-mean Gaussian noise with covariance R.
   * In this version, R is restricted to diagonal Gaussians (model parameter)
   *
   * @param p Previous state.
   * @param H Observation matrix.
   * @param z Measurement vector.
   * @param model Noise model (diagonal Gaussian).
   * @return Updated state.
   */
  State update(const State& p, const Matrix& H, const Vector& z,
               const SharedDiagonal& model) const;

  /**
   * Update the Kalman filter with a measurement using a full covariance matrix.
   * @param p Previous state.
   * @param H Observation matrix.
   * @param z Measurement vector.
   * @param R Full covariance matrix.
   * @return Updated state.
   */
  State updateQ(const State& p, const Matrix& H, const Vector& z,
                const Matrix& R) const;

  /**
   * Return the dimensionality of the state.
   * @return Dimensionality of the state.
   */
  size_t dim() const { return n_; }
};

}  // namespace gtsam
