/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    ManifoldEKF.h
 * @brief   Extended Kalman Filter base class on a generic manifold M
 *
 * This file defines the ManifoldEKF class template for performing prediction
 * and update steps of an Extended Kalman Filter on states residing in a
 * differentiable manifold. It relies on the manifold's retract and
 * localCoordinates operations.
 *
 * @date    April 24, 2025
 * @authors Scott Baker, Matt Kielo, Frank Dellaert
 */

#pragma once

#include <gtsam/base/Manifold.h>  // Include for traits and IsManifold
#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/base/Vector.h>

#include <Eigen/Dense>
#include <stdexcept>
#include <string>
#include <type_traits>

namespace gtsam {

/**
 * @class ManifoldEKF
 * @brief Extended Kalman Filter on a generic manifold M
 *
 * @tparam M  Manifold type (must satisfy Manifold concept).
 *
 * This filter maintains a state X in the manifold M and covariance P in the
 * tangent space at X.
 * Prediction requires providing the predicted next state and the state
 * transition Jacobian F. Updates apply a measurement function h and correct the
 * state using the tangent space error.
 */
template <typename M>
class ManifoldEKF {
 public:
  /// Compile-time dimension of the manifold M.
  static constexpr int Dim = traits<M>::dimension;

  /// Tangent vector type for the manifold M.
  using TangentVector = typename traits<M>::TangentVector;
  /// Covariance matrix type (P, Q).
  using Covariance = Eigen::Matrix<double, Dim, Dim>;
  /// State transition Jacobian type (F).
  using Jacobian = Eigen::Matrix<double, Dim, Dim>;

  /**
   * Constructor: initialize with state and covariance.
   * @param X0 Initial state on manifold M.
   * @param P0 Initial covariance in the tangent space at X0
   */
  ManifoldEKF(const M& X0, const Covariance& P0) : X_(X0) {
    static_assert(IsManifold<M>::value,
                  "Template parameter M must be a GTSAM Manifold.");

    n_ = traits<M>::GetDimension(X0);
    if constexpr (Dim == Eigen::Dynamic) {
      // Validate dimensions of initial covariance P0.
      if (P0.rows() != n_ || P0.cols() != n_) {
        throw std::invalid_argument(
            "ManifoldEKF: Initial covariance P0 dimensions (" +
            std::to_string(P0.rows()) + "x" + std::to_string(P0.cols()) +
            ") do not match state's tangent space dimension (" +
            std::to_string(n_) + ").");
      }
      I_ = Jacobian::Identity(n_, n_);
    } else {
      I_ = Jacobian::Identity();
    }

    P_ = P0;
  }

  virtual ~ManifoldEKF() = default;

  /// @return current state estimate on manifold M.
  const M& state() const { return X_; }

  /// @return current covariance estimate.
  const Covariance& covariance() const { return P_; }

  /// @return runtime dimension of the manifold.
  int dimension() const { return n_; }

  /**
   * Basic predict step: Updates state and covariance given the predicted next
   * state and the state transition Jacobian F.
   * This overload expects a **discrete-time** process covariance Q already
   * scaled for the step being applied.
   *   X_{k+1} = X_next
   *   P_{k+1} = F P_k F^T + Q
   * where F = d(local(X_{k+1})) / d(local(X_k)) is the Jacobian of the
   * state transition in local coordinates around X_k.
   *
   * @param X_next The predicted state at time k+1 on manifold M.
   * @param F The state transition Jacobian.
   * @param Q Process noise covariance matrix.
   */
  void predict(const M& X_next, const Jacobian& F, const Covariance& Q) {
    if constexpr (Dim == Eigen::Dynamic) {
      if (F.rows() != n_ || F.cols() != n_ || Q.rows() != n_ ||
          Q.cols() != n_) {
        throw std::invalid_argument(
            "ManifoldEKF::predict: Dynamic F/Q dimensions must match state "
            "dimension " +
            std::to_string(n_) + ".");
      }
    }

    X_ = X_next;
    P_ = F * P_ * F.transpose() + Q;
  }

  /// Kalman gain K = P H^T S^-1.
  template <typename HMatrix, typename RMatrix>
  auto KalmanGain(const HMatrix& H, const RMatrix& R) const {
    auto S = H * P_ * H.transpose() + R;  // Innovation covariance
    return P_ * H.transpose() * S.inverse();
  }

  /// Joseph-form covariance update in the current tangent space using a
  /// precomputed gain.
  template <typename GainMatrix, typename HMatrix, typename RMatrix>
  void JosephUpdate(const GainMatrix& K, const HMatrix& H, const RMatrix& R) {
    Jacobian I_KH = I_ - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
  }

  /**
   * Measurement update: Corrects the state and covariance using a
   * pre-calculated predicted measurement and its Jacobian.
   *
   * @tparam Measurement type of the measurement space.
   * @param prediction Predicted measurement.
   * @param H Jacobian of the measurement function h.
   * @param z Observed measurement.
   * @param R Measurement noise covariance.
   * @param performReset If true (default), performs a reset (transport) after
   * update; otherwise, just retracts the state.
   */
  template <typename Measurement>
  void update(
      const Measurement& prediction,
      const Eigen::Matrix<double, traits<Measurement>::dimension, Dim>& H,
      const Measurement& z,
      const Eigen::Matrix<double, traits<Measurement>::dimension,
                          traits<Measurement>::dimension>& R,
      bool performReset = true) {
    static constexpr int MeasDim = traits<Measurement>::dimension;

    // Innovation: y = h(x_pred) - z. In tangent space: local(z, h(x_pred))
    // NOTE: we use the `z_hat - z` sign convention, NOT `z - z_hat`.
    typename traits<Measurement>::TangentVector innovation =
        traits<Measurement>::Local(z, prediction);

    // Kalman Gain: K = P H^T S^-1
    // K will be Eigen::Matrix<double, Dim, MeasDim>
    Eigen::Matrix<double, Dim, MeasDim> K = KalmanGain(H, R);

    // Correction vector in tangent space of M: delta_xi = K * innovation
    const TangentVector delta_xi =
        -K * innovation;  // delta_xi is Dim x 1 (or n_ x 1 if dynamic)

    // --- Update covariance in the tangent space at the current state
    this->JosephUpdate(K, H, R);

    // Update state using retract/ transport or just retract
    if (performReset)
      reset(delta_xi);
    else
      X_ = traits<M>::Retract(X_, delta_xi);
  }

  /**
   * Measurement update: Corrects the state and covariance using a measurement
   * model function.
   *
   * @tparam Measurement type of the measurement space.
   * @tparam MeasurementFunction Functor/lambda providing measurement+Jacobian.
   * @param h Measurement model function.
   * @param z Observed measurement.
   * @param R Measurement noise covariance.
   * @param performReset If true (default), transport covariance after retract.
   */
  template <typename Measurement, typename MeasurementFunction>
  void update(MeasurementFunction&& h, const Measurement& z,
              const Eigen::Matrix<double, traits<Measurement>::dimension,
                                  traits<Measurement>::dimension>& R,
              bool performReset = true) {
    static_assert(IsManifold<Measurement>::value,
                  "Template parameter Measurement must be a GTSAM Manifold.");

    // Predict measurement and get Jacobian H = dh/dlocal(X)
    Matrix H(traits<Measurement>::GetDimension(z), n_);
    Measurement prediction = h(X_, H);

    // Call the other update function
    update<Measurement>(prediction, H, z, R, performReset);
  }

  /**
   * Convenience bridge for wrappers: vector measurement update calling
   * update<Vector>. This overload exists to avoid templates in wrappers. It
   * validates sizes and forwards to the templated update with Measurement =
   * gtsam::Vector (dynamic size).
   * @param prediction Predicted measurement vector.
   * @param H Measurement Jacobian matrix.
   * @param z Observed measurement vector.
   * @param R Measurement noise covariance matrix.
   * @param performReset If true (default), transport covariance after retract.
   */
  void updateWithVector(const gtsam::Vector& prediction, const Matrix& H,
                        const gtsam::Vector& z, const Matrix& R,
                        bool performReset = true) {
    validateInputs(prediction, H, z, R);
    update<Vector>(prediction, H, z, R, performReset);
  }

  /**
   * Reset step: retract the state by a tangent perturbation and, if available,
   * transport the covariance from the old tangent space to the new tangent
   * space.
   *
   * If the retract supports a Jacobian argument, we compute B and update
   * P <- B P B^T. Otherwise, we leave the covariance unchanged.
   */
  void reset(const TangentVector& eta) {
    if constexpr (HasRetractJacobian<M>::value) {
      Jacobian B;
      if constexpr (Dim == Eigen::Dynamic) B.resize(n_, n_);
      X_ = traits<M>::Retract(X_, eta, &B);
      P_ = B * P_ * B.transpose();
    } else {
      X_ = traits<M>::Retract(X_, eta);
      // Covariance unchanged when Jacobian is not available.
    }
  }

 protected:
  /// Validate inputs to update.
  void validateInputs(const gtsam::Vector& prediction, const Matrix& H,
                      const gtsam::Vector& z, const Matrix& R) {
    const int m = static_cast<int>(prediction.size());
    if (static_cast<int>(z.size()) != m) {
      throw std::invalid_argument(
          "ManifoldEKF::updateWithVector: prediction and z must have same "
          "length.");
    }
    if (H.rows() != m || H.cols() != n_) {
      throw std::invalid_argument(
          "ManifoldEKF::updateWithVector: H must be m x n where m = "
          "measurement size and n = state dimension.");
    }
    if (R.rows() != m || R.cols() != m) {
      throw std::invalid_argument(
          "ManifoldEKF::updateWithVector: R must be m x m where m = "
          "measurement size.");
    }
  }

 protected:
  M X_;           ///< Manifold state estimate.
  Covariance P_;  ///< Covariance (Eigen::Matrix<double, Dim, Dim>).
  Jacobian I_;    ///< Identity matrix sized to the state dimension.
  int n_;         ///< Runtime tangent space dimension of M.

 private:
  // Detection helper: check if traits<T>::Retract(x, v, Jacobian*) is valid.
  template <typename T, typename = void>
  struct HasRetractJacobian : std::false_type {};
  template <typename T>
  struct HasRetractJacobian<
      T, std::void_t<decltype(traits<T>::Retract(
             std::declval<const T&>(),
             std::declval<const typename traits<T>::TangentVector&>(),
             (Jacobian*)nullptr))>> : std::true_type {};
};

}  // namespace gtsam
