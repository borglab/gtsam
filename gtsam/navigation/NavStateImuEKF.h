/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  NavStateImuEKF.h
 * @brief Extended Kalman Filter for IMU-driven NavState on SE(3).
 *
 * @date  August 2025
 * @authors Derek Benham, Frank Dellaert
 */

#pragma once

#include <gtsam/navigation/LeftLinearEKF.h>  // Include the base class
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/PreintegrationParams.h>

namespace gtsam {

/// Specialized EKF for IMU-driven NavState on SE_2(3)
class GTSAM_EXPORT NavStateImuEKF : public LeftLinearEKF<NavState> {
 public:
  using Base = LeftLinearEKF<NavState>;
  using TangentVector = typename Base::TangentVector;  // Vector9
  using Jacobian = typename Base::Jacobian;            // 9x9
  using Covariance = typename Base::Covariance;        // 9x9

  /**
   * Construct with initial state/covariance and preintegration params (for
   * gravity and IMU covariances)
   * @param X0 Initial NavState.
   * @param P0 Initial covariance in tangent space at X0.
   * @param params Preintegration parameters providing gravity and options.
   */
  NavStateImuEKF(const NavState& X0, const Covariance& P0,
                 const std::shared_ptr<PreintegrationParams>& params);

  /// Calculate W (gravity-only left composition, world-frame increments)
  static NavState Gravity(const Vector3& g_n, double dt) {
    return {Rot3(), g_n * (0.5 * dt * dt), g_n * dt};
  }

  /// Calculate U from raw IMU (no gravity): body-frame increments
  /// We do an explicit closed-form integration based on SO(3) kernels
  static NavState Imu(const Vector3& omega_b, const Vector3& f_b, double dt);

  /**
   * @brief Compute the dynamics of the system.
   *
   * This function computes the next state of the system based on the current
   * state, gravity, body angular velocity, body specific force, and time step.
   * The dynamics are defined as:
   * X_{k+1} = f(X_k; g, omega_b, f_b, dt)
   *         = W(g, dt) \phi_dt(X_k) U(omega_b, f_b, dt)
   * where W, \phi, and U are the gravity, (autonomous) position update, and
   * IMU increment functions, respectively.
   *
   * @param g_n Gravity vector in the navigation frame.
   * @param X Current NavState.
   * @param omega_b Body angular velocity measurement (rad/s).
   * @param f_b Body specific force measurement (m/s^2).
   * @param dt Time step in seconds.
   * @param A Optional Jacobian of the dynamics with respect to the state.
   * @return The next NavState after applying the dynamics.
   */
  static NavState Dynamics(const Vector3& g_n, const NavState& X,
                           const Vector3& omega_b, const Vector3& f_b,
                           double dt, OptionalJacobian<9, 9> A = {});

  /**
   * @brief Predict the next state using gyro and accelerometer measurements.
   *
   * This method updates the state of the system based on the provided body
   * angular velocity (omega_b) and body specific force (f_b)
   * measurements over a given time step.
   *
   * @param omega_b Body angular velocity measurement (rad/s).
   * @param f_b Body specific force measurement (m/s^2).
   * @param dt Time step in seconds.
   */
  void predict(const Vector3& omega_b, const Vector3& f_b, double dt);

  /// Accessors
  const std::shared_ptr<PreintegrationParams>& params() const;
  const Vector3& gravity() const;
  const Covariance& processNoise() const;

 private:
  std::shared_ptr<PreintegrationParams> params_;
  Covariance Q_ = Covariance::Zero();
};

}  // namespace gtsam