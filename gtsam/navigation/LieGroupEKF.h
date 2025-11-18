/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  LieGroupEKF.h
 * @brief   Extended Kalman Filter derived class for Lie groups G.
 *
 * This file defines the LieGroupEKF class template, inheriting from
 * ManifoldEKF, for performing EKF steps specifically on states residing in a
 * Lie group. It provides predict methods with state-dependent dynamics
 * functions. Please use the InvariantEKF class for prediction via group
 * composition.
 *
 * @date  April 24, 2025
 * @authors Scott Baker, Matt Kielo, Frank Dellaert
 */

#pragma once

#include <gtsam/base/Lie.h>  // Include for Lie group traits and operations
#include <gtsam/base/VectorSpace.h>        // Matrix traits
#include <gtsam/navigation/ManifoldEKF.h>  // Include the base class

#include <Eigen/Dense>
#include <functional>  // For std::function
#include <type_traits>

namespace gtsam {

/**
 * @class LieGroupEKF
 * @brief Extended Kalman Filter on a Lie group G, derived from ManifoldEKF
 *
 * @tparam G Lie group type (must satisfy LieGroup concept).
 *
 * This filter specializes ManifoldEKF for Lie groups, offering predict methods
 * with state-dependent dynamics functions.
 * Use the InvariantEKF class for prediction via group composition.
 * For details on how static and dynamic dimensions are handled, please refer to
 * the `ManifoldEKF` class documentation.
 */
template <typename G>
class LieGroupEKF : public ManifoldEKF<G> {
 public:
  using Base = ManifoldEKF<G>;           ///< Base class type
  static constexpr int Dim = Base::Dim;  ///< Compile-time dimension of G.
  using TangentVector =
      typename Base::TangentVector;  ///< Tangent vector type for G.
  /// Jacobian for group operations (Adjoint, Expmap derivatives, F).
  using Jacobian = typename Base::Jacobian;  // Eigen::Matrix<double, Dim, Dim>
  using Covariance =
      typename Base::Covariance;  // Eigen::Matrix<double, Dim, Dim>

  /**
   * Constructor: initialize with state and covariance.
   * @param X0 Initial state on Lie group G.
   * @param P0 Initial covariance in the tangent space at X0.
   */
  LieGroupEKF(const G& X0, const Covariance& P0) : Base(X0, P0) {
    static_assert(IsLieGroup<G>::value,
                  "Template parameter G must be a GTSAM Lie Group");
  }

  /**
   * SFINAE check for correctly typed state-dependent dynamics function.
   * Signature: TangentVector f(const G& X, OptionalJacobian<Dim, Dim> Df)
   * Df = d(xi)/d(local(X))
   */
  template <typename Dynamics>
  using enable_if_dynamics =
      std::enable_if_t<!std::is_convertible_v<Dynamics, TangentVector> &&
                       std::is_invocable_r_v<TangentVector, Dynamics, const G&,
                                             OptionalJacobian<Dim, Dim>&>>;

  /**
   * Predict mean and Jacobian A with state-dependent dynamics:
   *   xi = f(X_k, Df)           (Compute tangent vector dynamics and Jacobian
   * Df) U = Expmap(xi * dt, Dexp) (Compute motion increment U and Expmap
   * Jacobian Dexp) X_{k+1} = X_k * U         (Predict next state) A =
   * Ad_{U^{-1}} + Dexp * Df * dt (Compute full state transition Jacobian)
   *
   * @tparam Dynamics Functor signature: TangentVector f(const G&,
   * OptionalJacobian<Dim,Dim>&)
   * @param f Dynamics functor returning tangent vector xi and its Jacobian Df
   * w.r.t. local(X).
   * @param dt Time step.
   * @param A OptionalJacobian to store the computed state transition Jacobian
   * A.
   * @return Predicted state X_{k+1}.
   */
  template <typename Dynamics, typename = enable_if_dynamics<Dynamics>>
  G predictMean(Dynamics&& f, double dt,
                OptionalJacobian<Dim, Dim> A = {}) const {
    Jacobian Df, Dexp;  // Eigen::Matrix<double, Dim, Dim>

    if constexpr (std::is_same_v<G, Matrix>) {
      // Specialize to Matrix case as its Expmap is not defined.
      TangentVector xi = f(this->X_, A ? &Df : nullptr);
      const Matrix nextX = traits<Matrix>::Retract(
          this->X_, xi * dt, A ? &Dexp : nullptr);  // just addition
      if (A) {
        const Matrix I_n = Matrix::Identity(this->n_, this->n_);
        *A = I_n + Dexp * Df * dt;  // AdjointMap is always identity for Matrix
      }
      return nextX;
    } else {
      TangentVector xi =
          f(this->X_, A ? &Df : nullptr);  // xi and Df = d(xi)/d(localX)
      G U = traits<G>::Expmap(xi * dt, A ? &Dexp : nullptr);
      if (A) {
        // State transition Jacobian for left-invariant EKF:
        *A = traits<G>::Inverse(U).AdjointMap() + Dexp * Df * dt;
      }
      return this->X_.compose(U);
    }
  }

  /**
   * Predict step with state-dependent dynamics:
   * Uses predictMean to compute X_{k+1} and A, then updates covariance.
   *   X_{k+1}, A = predictMean(f, dt)
   *   P_{k+1} = A P_k A^T + Q
   *
   * @tparam Dynamics Functor signature: TangentVector f(const G&,
   * OptionalJacobian<Dim,Dim>&)
   * @param f Dynamics functor.
   * @param dt Time step.
   * @param Q Process noise covariance.
   */
  template <typename Dynamics, typename = enable_if_dynamics<Dynamics>>
  void predict(Dynamics&& f, double dt, const Covariance& Q) {
    Jacobian A;
    if constexpr (Dim == Eigen::Dynamic) {
      A.resize(this->n_, this->n_);
    }
    this->X_ = predictMean(std::forward<Dynamics>(f), dt, A);
    this->P_ = A * this->P_ * A.transpose() + Q;
  }

  /**
   * SFINAE check for state- and control-dependent dynamics function.
   * Signature: TangentVector f(const G& X, const Control& u,
   * OptionalJacobian<Dim, Dim> Df)
   */
  template <typename Control, typename Dynamics>
  using enable_if_full_dynamics = std::enable_if_t<
      std::is_invocable_r_v<TangentVector, Dynamics, const G&, const Control&,
                            OptionalJacobian<Dim, Dim>&>>;

  /**
   * Predict mean and Jacobian A with state and control input dynamics:
   * Wraps the dynamics function and calls the state-only predictMean.
   *   xi = f(X_k, u, Df)
   *
   * @tparam Control Control input type.
   * @tparam Dynamics Functor signature: TangentVector f(const G&, const
   * Control&, OptionalJacobian<Dim,Dim>&)
   * @param f Dynamics functor.
   * @param u Control input.
   * @param dt Time step.
   * @param A Optional pointer to store the computed state transition Jacobian
   * A.
   * @return Predicted state X_{k+1}.
   */
  template <typename Control, typename Dynamics,
            typename = enable_if_full_dynamics<Control, Dynamics>>
  G predictMean(Dynamics&& f, const Control& u, double dt,
                OptionalJacobian<Dim, Dim> A = {}) const {
    return predictMean(
        [&](const G& X, OptionalJacobian<Dim, Dim> Df) { return f(X, u, Df); },
        dt, A);
  }

  /**
   * Predict step with state and control input dynamics:
   * Wraps the dynamics function and calls the state-only predict.
   *   xi = f(X_k, u, Df)
   *
   * @tparam Control Control input type.
   * @tparam Dynamics Functor signature: TangentVector f(const G&, const
   * Control&, OptionalJacobian<Dim,Dim>&)
   * @param f Dynamics functor.
   * @param u Control input.
   * @param dt Time step.
   * @param Q Process noise covariance.
   */
  template <typename Control, typename Dynamics,
            typename = enable_if_full_dynamics<Control, Dynamics>>
  void predict(Dynamics&& f, const Control& u, double dt, const Covariance& Q) {
    return predict(
        [&](const G& X, OptionalJacobian<Dim, Dim> Df) { return f(X, u, Df); },
        dt, Q);
  }

  /**
   * Predict using a precomputed group increment U and its Jacobian J_UX.
   *
   * Contract:
   * - Input state X_k on G with covariance P_k in local coordinates
   * - Precomputed increment U = U(X_k) in G
   * - Jacobian J_UX = d(u_left)/d(local(X)) at X_k, where u_left = Log(U)
   * - Process noise Q expressed in the same coordinates as u_left
   *
   * Update performed:
   * - X_{k+1} = X_k ∘ U
   * - A = Ad_{U^{-1}} + J_UX
   * - P_{k+1} = A P_k A^T + Q
   *
   * Notes:
   * This API is intended for custom integrators that construct U(X) directly
   * (e.g., second-order kinematics), which may not be expressible as an Euler
   * step on the tangent used by predict/predictMean.
   */
  void predictWithCompose(const G& U, const Jacobian& J_UX,
                          const Covariance& Q) {
    Jacobian A_local;
    if constexpr (std::is_same_v<G, Matrix>) {
      const Matrix I_n = Matrix::Identity(this->n_, this->n_);
      A_local = I_n + J_UX;
      this->X_ = traits<Matrix>::Retract(this->X_, U);
    } else {
      A_local = traits<G>::Inverse(U).AdjointMap() + J_UX;
      this->X_ = this->X_.compose(U);
    }
    this->P_ = A_local * this->P_ * A_local.transpose() + Q;
  }

};  // LieGroupEKF

}  // namespace gtsam