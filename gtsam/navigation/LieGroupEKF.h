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
#include <gtsam/navigation/ManifoldEKF.h>  // Include the base class

#include <Eigen/Dense>
#include <functional>  // For std::function
#include <type_traits>
#include <utility>

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
 *
 * Update API: inherited from ManifoldEKF (`update(prediction, H, z, R)`,
 * `update(h, z, R)`, and `updateWithVector`).
 *
 * Noise convention:
 * - Overloads **without** `dt` (e.g., `predict(X_next, F, Q)` inherited from
 *   ManifoldEKF) expect `Q` to be a *discrete* covariance already scaled for
 *   the step being applied.
 * - Overloads **with** `dt` interpret `Q` as a *continuous-time* covariance.
 */
template <typename G>
class LieGroupEKF : public ManifoldEKF<G> {
 public:
  using This = LieGroupEKF<G>;
  using Base = ManifoldEKF<G>;           ///< Base class type
  static constexpr int Dim = Base::Dim;  ///< Compile-time dimension of G.

  using Jacobian = typename Base::Jacobian;            ///< Dim x Dim
  using Covariance = typename Base::Covariance;        ///< Dim x Dim
  using TangentVector = typename Base::TangentVector;  ///< Tangent vector type.

 private:
  /**
   * SFINAE check for correctly typed state-dependent dynamics function.
   * TangentVector f(const G& X, OptionalJacobian Df)
   */
  template <typename Dynamics>
  using enable_if_dynamics =
      std::enable_if_t<!std::is_convertible_v<Dynamics, TangentVector> &&
                       std::is_invocable_r_v<TangentVector, Dynamics, const G&,
                                             OptionalJacobian<Dim, Dim>&>>;

  /**
   * SFINAE check for state- and control-dependent dynamics function.
   * TangentVector f(const G& X, const Control& u, OptionalJacobian Df)
   */
  template <typename Control, typename Dynamics>
  using enable_if_full_dynamics = std::enable_if_t<
      std::is_invocable_r_v<TangentVector, Dynamics, const G&, const Control&,
                            OptionalJacobian<Dim, Dim>&>>;

  template <typename T, typename = void>
  struct has_adjoint_map : std::false_type {};

  template <typename T>
  struct has_adjoint_map<
      T, std::void_t<decltype(T::adjointMap(
             std::declval<typename traits<T>::TangentVector>()))>>
      : std::true_type {};

 public:
  /**
   * Constructor: initialize with state and covariance.
   * @param X0 Initial state on Lie group G.
   * @param P0 Initial covariance in the tangent space at X0.
   */
  LieGroupEKF(const G& X0, const Covariance& P0) : Base(X0, P0) {
    static_assert(IsLieGroup<G>::value,
                  "Template parameter G must be a GTSAM Lie Group");
  }

  /// Expose base class predict method,
  /// predict(const M& X_next, const Jacobian& F, const Covariance& Q)
  using Base::predict;

  /**
   * Compute the discrete-time transition matrix Φ corresponding to a
   * continuous-time linearization (Df) over time dt.
   *
   * @tparam K Truncation order for expm (K=1 for first-order).
   * @param xi Tangent increment (used only for Lie groups).
   * @param Df Jacobian of dynamics w.r.t. local coordinates.
   * @param dt Time step.
   * @param U Increment Expmap(xi * dt) (ignored for Matrix case).
   * @param Dexp Jacobian returned by Expmap (ignored for Matrix case).
   */
  template <size_t K = 1>
  Jacobian transitionMatrix(const TangentVector& xi, const Jacobian& Df,
                            double dt, const G& U, const Jacobian& Dexp) const {
    if constexpr (std::is_same_v<G, Matrix>) {
      (void)xi;
      (void)U;
      (void)Dexp;
      return expm(Df * dt, K);
    } else {
      if constexpr (K == 1) {
        // First-order Lie group transition matrix
        return traits<G>::Inverse(U).AdjointMap() + Dexp * Df * dt;
      } else {
        // Higher-order Lie group transition matrix via matrix exponential.
        //
        // GTSAM's LieGroupEKF uses a left-invariant error, i.e. we write the
        // true state as
        //
        //     X_true ≈ X_hat * Exp(η),
        //
        // and η lives in the tangent space at the identity. Even if the
        // *global* perturbation is constant, composing the nominal motion X_hat
        // with Exp(ξ dt) on the right will "drag" η along the group because G
        // is non-commutative. The Baker–Campbell–Hausdorff expansion shows that
        // the linearized error dynamics contain an extra term -ad_ξ η coming
        // from this non-commutativity, where ad_ξ(·) = [ξ, ·] is the Lie
        // bracket. With this convention, and a dynamics linearization Df =
        // ∂ξ/∂(local X), the continuous-time error satisfies approximately
        //
        //     dη/dt ≈ (Df - ad_ξ) η,
        //
        // and the corresponding discrete-time transition matrix is
        //
        //     Φ ≈ expm((Df - ad_ξ) dt).
        //
        // This is exactly what we implement below.
        static_assert(
            has_adjoint_map<G>::value,
            "transitionMatrix<K> requires G::adjointMap(xi) when K > 1.");
        Jacobian ad_xi = G::adjointMap(xi);
        const Matrix A = Df - ad_xi;
        return expm(A * dt, K);
      }
    }
  }

  /**
   * Predict mean and Jacobian Phi with state-dependent dynamics:
   *   xi = f(X_k, Df)           (tangent vector dynamics and Jacobian Df)
   *   U = Expmap(xi * dt, Dexp) (motion increment U and Expmap Jacobian Dexp)
   *   X_{k+1} = X_k * U         (Predict next state via compose)
   *   Phi = Ad_{U^{-1}} + Dexp * Df * dt (K=1 first-order Jacobian)
   *       expm((Df - ad(xi))dt) (K>1 matrix exponential discretization)
   *
   * @tparam K Truncation order for expm (K=1 for first-order).
   * @tparam Dynamics: TangentVector f(const G&, OptionalJacobian<Dim,Dim>&)
   * @param f Dynamics function computing tangent vector xi and its Jacobian Df.
   * @param dt Time step.
   * @param Phi OptionalJacobian to store the computed transition matrix Phi.
   * @return Predicted state X_{k+1}.
   */
  template <size_t K = 1, typename Dynamics,
            typename = enable_if_dynamics<Dynamics>>
  G predictMean(Dynamics&& f, double dt,
                OptionalJacobian<Dim, Dim> Phi = {}) const {
    Jacobian Df, Dexp;

    if constexpr (std::is_same_v<G, Matrix>) {
      TangentVector xi = f(this->X_, Phi ? &Df : nullptr);
      if (Phi) *Phi = expm(Df * dt, K);
      return traits<G>::Retract(this->X_, xi * dt);
    } else {
      TangentVector xi = f(this->X_, Phi ? &Df : nullptr);
      G U = traits<G>::Expmap(xi * dt, Phi ? &Dexp : nullptr);
      if (Phi) *Phi = transitionMatrix<K>(xi, Df, dt, U, Dexp);
      return traits<G>::Compose(this->X_, U);
    }
  }

  /**
   * Predict step with state-dependent dynamics:
   * Uses predictMean to compute X_{k+1} and Phi, then updates covariance.
   *   X_{k+1}, Phi = predictMean(f, dt)
   *   P_{k+1} = Phi P_k Phi^T + Q
   *
   * @tparam K Truncation order for expm (K=1 for first-order).
   * @tparam Dynamics Functor signature: TangentVector f(const G&,
   * OptionalJacobian<Dim,Dim>&)
   * @param f Dynamics functor.
   * @param dt Time step.
   * @param Q Continuous-time process noise covariance (will be scaled by dt).
   */
  template <size_t K = 1, typename Dynamics,
            typename = enable_if_dynamics<Dynamics>>
  void predict(Dynamics&& f, double dt, const Covariance& Q) {
    Jacobian Phi;
    if constexpr (Dim == Eigen::Dynamic) {
      Phi.resize(this->n_, this->n_);
    }
    G X_next = predictMean<K>(std::forward<Dynamics>(f), dt, Phi);
    predict(X_next, Phi,
            Q * dt);  // Q interpreted as continuous-time; discretize with dt
  }

  /**
   * Predict mean and Jacobian A with state and control input dynamics:
   * Wraps the dynamics function and calls the state-only predictMean.
   *   xi = f(X_k, u, Df)
   *
   * @tparam K Truncation order for expm (K=1 for first-order).
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
  template <size_t K = 1, typename Control, typename Dynamics,
            typename = enable_if_full_dynamics<Control, Dynamics>>
  G predictMean(Dynamics&& f, const Control& u, double dt,
                OptionalJacobian<Dim, Dim> Phi = {}) const {
    return predictMean<K>(
        [&](const G& X, OptionalJacobian<Dim, Dim> Df) { return f(X, u, Df); },
        dt, Phi);
  }

  /**
   * Predict step with state and control input dynamics:
   * Wraps the dynamics function and calls the state-only predict.
   *   xi = f(X_k, u, Df)
   *
   * @tparam K Truncation order for the matrix exponential (K=1 recovers
   * first-order).
   * @tparam Control Control input type.
   * @tparam Dynamics Functor signature: TangentVector f(const G&, const
   * Control&, OptionalJacobian<Dim,Dim>&)
   * @param f Dynamics functor.
   * @param u Control input.
   * @param dt Time step.
   * @param Q Continuous-time process noise covariance (will be scaled by dt).
   */
  template <size_t K = 1, typename Control, typename Dynamics,
            typename = enable_if_full_dynamics<Control, Dynamics>>
  void predict(Dynamics&& f, const Control& u, double dt, const Covariance& Q) {
    return predict<K>(
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
      const Matrix& I_n = this->I_;
      A_local = I_n + J_UX;
      this->X_ = traits<Matrix>::Retract(this->X_, U);
    } else {
      A_local = traits<G>::Inverse(U).AdjointMap() + J_UX;
      this->X_ = this->X_.compose(U);
    }
    this->P_ = A_local * this->P_ * A_local.transpose() + Q;
  }

  /// Update overloads follow ManifoldEKF.
  using Base::update;

};  // LieGroupEKF

}  // namespace gtsam
