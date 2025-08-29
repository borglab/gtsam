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
  * This file defines the InvariantEKF class template, inheriting from LieGroupEKF,
  * which specifically implements the Left-Invariant EKF formulation. It restricts
  * prediction methods to only those based on group composition (state-independent
  * motion models), hiding the state-dependent prediction variants from LieGroupEKF.
  *
  * @date    April 24, 2025
  * @authors Scott Baker, Matt Kielo, Frank Dellaert
  */

#pragma once

#include <gtsam/navigation/LieGroupEKF.h> // Include the base class
#include <gtsam/base/Lie.h> // For traits (needed for AdjointMap, Expmap)


namespace gtsam {

  /**
   * @class InvariantEKF
   * @brief Left-Invariant Extended Kalman Filter on a Lie group G.
   *
   * @tparam G Lie group type (must satisfy LieGroup concept).
   *
   * This filter inherits from LieGroupEKF but restricts the prediction interface
   * to only the left-invariant prediction methods:
   * 1. Prediction via group composition: `predict(const G& U, const Covariance& Q)`
   * 2. Prediction via tangent control vector: `predict(const TangentVector& u, double dt, const Covariance& Q)`
   *
   * The state-dependent prediction methods from LieGroupEKF are hidden.
   * The update step remains the same as in ManifoldEKF/LieGroupEKF.
   * For details on how static and dynamic dimensions are handled, please refer to
   * the `ManifoldEKF` class documentation.
   */
  template <typename G>
  class InvariantEKF : public LieGroupEKF<G> {
  public:
    using Base = LieGroupEKF<G>; ///< Base class type
    using TangentVector = typename Base::TangentVector; ///< Tangent vector type
    /// Jacobian for group-specific operations like AdjointMap. Eigen::Matrix<double, Dim, Dim>.
    using Jacobian = typename Base::Jacobian;
    /// Covariance matrix type. Eigen::Matrix<double, Dim, Dim>.
    using Covariance = typename Base::Covariance;


    /**
     * Constructor: forwards to LieGroupEKF constructor.
     * @param X0 Initial state on Lie group G.
     * @param P0 Initial covariance in the tangent space at X0.
     */
    InvariantEKF(const G& X0, const Covariance& P0) : Base(X0, P0) {}

    // We hide state-dependent predict methods from LieGroupEKF by only providing the
    // invariant predict methods below.

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
      }
      else {
        U = traits<G>::Expmap(u * dt);
      }
      predict(U, Q); // Call the group composition predict
    }

        /**
     * Predict step via tangent control vector and manual imu integration on NavState (SE_2(3)) group.
     * Gravity vector added to allow overloaded function and account for example code 
     * IEKF_NavstateExample.cpp that does not have gravity offset
     *
     * Motion propagation equations follow the formulation from:
     * Potokar et al., "Invariant Extended Kalman Filtering for Underwater Navigation"
     * Link to paper: https://ieeexplore.ieee.org/document/9444664
     * Source code can be found here: https://bitbucket.org/frostlab/underwateriekf/src/main/src/system.py
     * 
     *   Motion Propagation:
     *   R_new = R * Expmap(omega*dt)
     *   v_new = v + (R * accel + g) * dt   
     *   p_new = p + v * dt + ((R * accel + g) * dt^2) / 2
     *   X_ = NavState(Pose3(R_new, p_new), v_new);
     * 
     *   Covariance Update:
     *   A = motion linearization which is dependent on control u input
     *   P_ = exp(A) * P_ * exp(A).transpose() + exp(A) * Q * exp(A).transpose() * dt
     *   
     * @param u Tangent space control vector.
     * @param dt Time interval.
     * @param Q Process noise covariance matrix.
     * @param g Gravity vector
     * 
     * @authors Derek Benham
     */
    
    void predict(const TangentVector&u, double dt, const Covariance& Q, const Vector3 g) {
      const Rot3 R = this->X_.pose().rotation();
      const Vector3 v = this->X_.velocity();
      const Vector3 p = this->X_.pose().translation();
      
      Vector3 omega = u.template segment<3>(0);
      Vector3 accel = u.template segment<3>(6);
      
      // Motion propagation
      Rot3 dR = traits<Rot3>::Expmap(omega * dt);
      Rot3 R_new = R * dR; // R_new = R * \delta R
      Vector3 v_new = v + (R * accel + g) * dt;
      Vector3 p_new = p + v * dt + ((R * accel + g) * dt * dt) / 2;
      this->X_ = gtsam::NavState(Pose3(R_new, p_new), v_new);
      
      // Updating Covariance
      const Matrix3 zero = Matrix3::Zero();
      const Matrix3 I = Matrix3::Identity();

      Matrix3 w_cross;
      w_cross <<     0, -omega(2),  omega(1),
              omega(2),         0, -omega(0),
             -omega(1),  omega(0),         0;

      Matrix3 a_cross;
      a_cross <<     0, -accel(2),  accel(1),
              accel(2),         0, -accel(0),
             -accel(1),  accel(0),         0;

      // Left error A matrix
      Matrix A(9, 9);
      A << -w_cross,     zero,  zero,
           -a_cross, -w_cross,  zero,
               zero,        I,  -w_cross;

      // Compute expA = exp(A * dt), where A is the system's linearized dynamics matrix
      Matrix expA = (A * dt).exp();

      // Update the covariance with process noise
      // Pₖ₊₁ = expA * Pₖ * expAᵀ + expA * Q * expAᵀ * dt
      Matrix P_predict = expA * this->P_ * expA.transpose();
      Matrix Q_predict = expA * Q * expA.transpose() * dt;
      this->P_ = P_predict + Q_predict;
    }


  }; // InvariantEKF

} // namespace gtsam