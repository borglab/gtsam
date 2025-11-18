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
 *
 * **Handling Static and Dynamic Dimensions:**
 * The filter supports manifolds M with either a compile-time fixed dimension or
 * a runtime dynamic dimension. This is determined by
 * `gtsam::traits<M>::dimension`.
 * - If `dimension` is an integer (e.g., 3, 6), it's a fixed-size manifold.
 * - If `dimension` is `Eigen::Dynamic`, it's a dynamically-sized manifold. In
 * this case, `gtsam::traits<M>::GetDimension(const M&)` must be available to
 * retrieve the actual dimension at runtime. The internal protected member `n_`
 * stores this runtime dimension. Covariance matrices (e.g., `P_`, method
 * argument `Q`) and Jacobians (e.g., method argument `F`) are typed using
 * `Covariance` and `Jacobian` typedefs, which are specializations of
 * `Eigen::Matrix<double, Dim, Dim>`, where `Dim` is `traits<M>::dimension`.
 * For dynamically-sized manifolds (`Dim == Eigen::Dynamic`), these Eigen types
 * represent dynamically-sized matrices.
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

    if constexpr (Dim == Eigen::Dynamic) {
      n_ = traits<M>::GetDimension(X0);
      // Validate dimensions of initial covariance P0.
      if (P0.rows() != n_ || P0.cols() != n_) {
        throw std::invalid_argument(
            "ManifoldEKF: Initial covariance P0 dimensions (" +
            std::to_string(P0.rows()) + "x" + std::to_string(P0.cols()) +
            ") do not match state's tangent space dimension (" +
            std::to_string(n_) + ").");
      }
    } else {
      n_ = Dim;
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
   * Basic predict step: Updates state and covariance given the predicted
   * next state and the state transition Jacobian F.
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

  /**
   * Measurement update: Corrects the state and covariance using a
   * pre-calculated predicted measurement and its Jacobian.
   *
   * @tparam Measurement Type of the measurement vector (e.g., VectorN<m>,
   * Vector).
   * @param prediction Predicted measurement.
   * @param H Jacobian of the measurement function h w.r.t. local(X), H =
   * dh/dlocal(X).
   * @param z Observed measurement.
   * @param R Measurement noise covariance.
   */
  template <typename Measurement>
  void update(
      const Measurement& prediction,
      const Eigen::Matrix<double, traits<Measurement>::dimension, Dim>& H,
      const Measurement& z,
      const Eigen::Matrix<double, traits<Measurement>::dimension,
                          traits<Measurement>::dimension>& R) {
    static_assert(IsManifold<Measurement>::value,
                  "Template parameter Measurement must be a GTSAM Manifold for "
                  "LocalCoordinates.");

    static constexpr int MeasDim = traits<Measurement>::dimension;

    if constexpr (MeasDim == Eigen::Dynamic) {
      int m_runtime = traits<Measurement>::GetDimension(z);
      if (traits<Measurement>::GetDimension(prediction) != m_runtime) {
        throw std::invalid_argument(
            "ManifoldEKF::update: Dynamic measurement 'prediction' and 'z' "
            "have different dimensions.");
      }
      if (H.rows() != m_runtime || H.cols() != n_ || R.rows() != m_runtime ||
          R.cols() != m_runtime) {
        throw std::invalid_argument(
            "ManifoldEKF::update: Jacobian H or Noise R dimensions mismatch "
            "for dynamic measurement.");
      }
    } else {
      if constexpr (Dim == Eigen::Dynamic) {
        if (H.cols() != n_) {
          throw std::invalid_argument(
              "ManifoldEKF::update: Jacobian H columns must match state "
              "dimension " +
              std::to_string(n_) + ".");
        }
      }
    }

    // Innovation: y = z - h(x_pred). In tangent space: local(h(x_pred), z)
    typename traits<Measurement>::TangentVector innovation =
        traits<Measurement>::Local(prediction, z);

    // Innovation covariance: S = H P H^T + R
    // S will be Eigen::Matrix<double, MeasDim, MeasDim>
    Eigen::Matrix<double, MeasDim, MeasDim> S = H * P_ * H.transpose() + R;

    // Kalman Gain: K = P H^T S^-1
    // K will be Eigen::Matrix<double, Dim, MeasDim>
    Eigen::Matrix<double, Dim, MeasDim> K = P_ * H.transpose() * S.inverse();

    // Correction vector in tangent space of M: delta_xi = K * innovation
    const TangentVector delta_xi =
        K * innovation;  // delta_xi is Dim x 1 (or n_ x 1 if dynamic)

    // Update state using retract: X_new = retract(X_old, delta_xi)
    X_ = traits<M>::Retract(X_, delta_xi);

    // Update covariance using Joseph form for numerical stability
    Jacobian I_n;  // Eigen::Matrix<double, Dim, Dim>
    if constexpr (Dim == Eigen::Dynamic) {
      I_n = Jacobian::Identity(n_, n_);
    } else {
      I_n = Jacobian::Identity();
    }

    // I_KH will be Eigen::Matrix<double, Dim, Dim>
    Jacobian I_KH = I_n - K * H;
    P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();
  }

  /**
   * Measurement update: Corrects the state and covariance using a measurement
   * model function.
   *
   * @tparam Measurement Type of the measurement vector.
   * @tparam MeasurementFunction Functor/lambda providing measurement and its
   * Jacobian. Signature: `Measurement h(const M& x, Jac& H_jacobian)` where H =
   * d(h)/d(local(X)).
   * @param h Measurement model function.
   * @param z Observed measurement.
   * @param R Measurement noise covariance.
   */
  template <typename Measurement, typename MeasurementFunction>
  void update(MeasurementFunction&& h, const Measurement& z,
              const Eigen::Matrix<double, traits<Measurement>::dimension,
                                  traits<Measurement>::dimension>& R) {
    static_assert(IsManifold<Measurement>::value,
                  "Template parameter Measurement must be a GTSAM Manifold.");

    static constexpr int MeasDim = traits<Measurement>::dimension;

    int m_runtime;
    if constexpr (MeasDim == Eigen::Dynamic) {
      m_runtime = traits<Measurement>::GetDimension(z);
    } else {
      m_runtime = MeasDim;
    }

    // Predict measurement and get Jacobian H = dh/dlocal(X)
    Matrix H(m_runtime, n_);
    Measurement prediction = h(X_, H);

    // Call the other update function
    update(prediction, H, z, R);
  }

  /// Convenience bridge for wrappers: vector measurement update calling
  /// update<Vector>. This overload exists to avoid templates in wrappers. It
  /// validates sizes and forwards to the templated update with Measurement =
  /// gtsam::Vector (dynamic size).
  void updateWithVector(const gtsam::Vector& prediction, const Matrix& H,
                        const gtsam::Vector& z, const Matrix& R) {
    // Basic dimension checks for dynamic-sized measurement
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

    // Forward to templated update with Measurement = Vector
    this->template update<Vector>(prediction, H, z, R);
  }

 protected:
  M X_;           ///< Manifold state estimate.
  Covariance P_;  ///< Covariance (Eigen::Matrix<double, Dim, Dim>).
  int n_;         ///< Runtime tangent space dimension of M.
};

}  // namespace gtsam