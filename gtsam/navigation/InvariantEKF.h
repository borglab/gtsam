/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    InvariantEKF.h
 * @brief   Left-Invariant Extended Kalman Filter implementation.
 *
 * This file defines the InvariantEKF class template, inheriting from
 * LieGroupEKF, which specifically implements the Left-Invariant EKF
 * formulation. It restricts prediction methods to only those based on group
 * composition (state-independent motion models), hiding the state-dependent
 * prediction variants from LieGroupEKF.
 *
 * @date    April 24, 2025
 * @authors Scott Baker, Matt Kielo, Frank Dellaert
 */

#pragma once

#include <gtsam/base/Lie.h>  // For traits (needed for AdjointMap, Expmap)
#include <gtsam/navigation/LeftLinearEKF.h>  // Include the base class

namespace gtsam {

/**
 * @class InvariantEKF
 * @brief Left-Invariant Extended Kalman Filter on a Lie group G.
 *
 * @tparam G Lie group type (must satisfy LieGroup concept).
 *
 * This filter inherits from LeftLinearEKF but restricts the prediction
 * interface to only the left-invariant prediction methods:
 * 1. Prediction via group composition: `predict(const G& U, const Covariance&
 * Q)`
 * 2. Prediction via tangent control vector: `predict(const TangentVector& u,
 * double dt, const Covariance& Q)`
 *
 * The state-dependent prediction methods from LeftLinearEKF are hidden.
 * The update step remains the same as in ManifoldEKF/LeftLinearEKF.
 * For details on how static and dynamic dimensions are handled, please refer to
 * the `ManifoldEKF` class documentation.
 */
template <typename G>
class InvariantEKF : public LeftLinearEKF<G> {
 public:
  using Base = LeftLinearEKF<G>;         ///< Base class type
  static constexpr int Dim = Base::Dim;  ///< Compile-time dimension of G.
  using TangentVector = typename Base::TangentVector;  ///< Tangent vector type
  /// Jacobian for group-specific operations like AdjointMap.
  /// Eigen::Matrix<double, Dim, Dim>.
  using Jacobian = typename Base::Jacobian;
  /// Covariance matrix type. Eigen::Matrix<double, Dim, Dim>.
  using Covariance = typename Base::Covariance;

  /**
   * Constructor: forwards to LeftLinearEKF constructor.
   * @param X0 Initial state on Lie group G.
   * @param P0 Initial covariance in the tangent space at X0.
   */
  InvariantEKF(const G& X0, const Covariance& P0) : Base(X0, P0) {}

  /**
   * Dynamics with W=I.
   * Returns φ(X) · U, and optional Jacobian A = Ad_{U^{-1}} Φ,  Φ := dφ|_e.
   */
  static G Dynamics(const G& X, const G& U, OptionalJacobian<Dim, Dim> A = {}) {
    if (A) {
      const G U_inv = traits<G>::Inverse(U);
      *A = traits<G>::AdjointMap(U_inv);
    }
    return traits<G>::Compose(X, U);
  }

  // We hide state-dependent predict methods from LeftLinearEKF by only
  // providing the invariant predict methods below.

  /**
   * Predict step via group composition (Left-Invariant):
   *   X_{k+1} = X_k * U
   *   P_{k+1} = Ad_{U^{-1}} P_k Ad_{U^{-1}}^T + Q
   * where Ad_{U^{-1}} is the Adjoint map of U^{-1}.
   *
   * @param U Lie group element representing the motion increment.
   * @param Q Process noise covariance.
   */
  void predict(const G& U, const Covariance& Q) {
    this->X_ = traits<G>::Compose(this->X_, U);
    const G U_inv = traits<G>::Inverse(U);
    const Jacobian A = traits<G>::AdjointMap(U_inv);
    // P_ is Covariance. A is Jacobian. Q is Covariance.
    // All are Eigen::Matrix<double,Dim,Dim>.
    this->P_ = A * this->P_ * A.transpose() + Q;
  }

  /**
   * Predict step via tangent control vector:
   *   U = Expmap(u * dt)
   * Then calls predict(U, Q).
   *
   * @param u Tangent space control vector.
   * @param dt Time interval.
   * @param Q Process noise covariance matrix.
   */
  void predict(const TangentVector& u, double dt, const Covariance& Q) {
    G U;
    if constexpr (std::is_same_v<G, Matrix>) {
      // Specialize to Matrix case as its Expmap is not defined.
      const Matrix& X = static_cast<const Matrix&>(this->X_);
      U.resize(X.rows(), X.cols());
      Eigen::Map<Vector>(static_cast<Matrix&>(U).data(), U.size()) = u * dt;
    } else {
      U = traits<G>::Expmap(u * dt);
    }
    predict(U, Q);  // Call the group composition predict
  }

  /**
   * General dynamics.
   * Returns W · X · U, and optional Jacobian A = Ad_{U^{-1}}
   */
  static G Dynamics(const G& W, const G& X, const G& U,
                    OptionalJacobian<Dim, Dim> A = {}) {
    return traits<G>::Compose(W, Dynamics(X, U, A));  // A is independent of W
  }

  /**
   * Predict step via left and right group composition (Left-Invariant):
   *   X_{k+1} = W * X_k * U
   *   P_{k+1} = Ad_{U^{-1}} P_k Ad_{U^{-1}}^T + Q
   * where Ad_{U^{-1}} is the Adjoint map of U^{-1}.
   *
   * @param W Lie group element representing the motion increment in world
   * frame.
   * @param U Lie group element representing the motion increment in body frame.
   * @param Q Process noise covariance.
   */
  void predict(const G& W, const G& U, const Covariance& Q) {
    predict(U, Q);  // First apply U
    // Then apply W on the left
    this->X_ = traits<G>::Compose(W, this->X_);
  }

};  // InvariantEKF

}  // namespace gtsam