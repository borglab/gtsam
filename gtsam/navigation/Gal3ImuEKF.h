/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  Gal3ImuEKF.h
 * @brief (Invariant) Extended Kalman Filter for IMU-driven Gal3
 * We use an invariant Kalman Filter on the Gal3 Lie group to propagate from one
 * state to another. X_(k+1) = W*X*U where X(k) ∈ Gal3 follows format of [R, v,
 * p; 0, 1, dt; 0, 0, 1]
 *
 * W is the gravity matrix that transforms body to world frame where
 * W ∈ Gal3= [I_3, g * dt, -0.5*g*dt^2
 * 0, 1, -dt
 * 0, 0, 1]
 *
 * U ∈ Gal3 is the control matrix where
 * U = [dR, f_b*dt, f_b*0.5*dt^2
 * 0, 1, dt
 * 0, 0, 1]
 * @date  September 2025
 * @authors Scott Baker
 */

#pragma once

#include <gtsam/geometry/Gal3.h>
#include <gtsam/navigation/InvariantEKF.h>  // Include the base class
#include <gtsam/navigation/PreintegrationParams.h>

namespace gtsam {

/// Specialized EKF for IMU-driven on Gal3
class GTSAM_EXPORT Gal3ImuEKF : public InvariantEKF<Gal3> {
 public:
  using Base = InvariantEKF<Gal3>;
  using TangentVector = typename Base::TangentVector;  // Vector10
  using Jacobian = typename Base::Jacobian;            // 10x10
  using Covariance = typename Base::Covariance;        // 10x10

  /// The Gal3 EKF has three modes of operation
  enum Mode {
    NO_TIME,  ///< Do not track time, state remains in NavState sub-group
    TRACK_TIME_NO_COVARIANCE,  ///< Track time, but not its covariance (default)
    TRACK_TIME_WITH_COVARIANCE,  ///< Track time and its covariance.
  };

  /**
   * Construct with initial state/covariance and preintegration params (for
   * gravity and IMU covariances)
   * @param X0 Initial Gal3.
   * @param P0 Initial covariance in tangent space at X0.
   * @param params Preintegration parameters providing gravity and options.
   */
  Gal3ImuEKF(const Gal3& X0, const Covariance& P0,
             const std::shared_ptr<PreintegrationParams>& params,
             Mode mode = TRACK_TIME_NO_COVARIANCE);

  /// Calculate gravity-only left composition, world-frame increments
  /// p = +1/2 g dt^2, v = g dt, t = 0
  static Gal3 Gravity(const Vector3& g_n, double dt) {
    return {Rot3(), g_n * (0.5 * dt * dt), g_n * dt, 0.0};
  }

  /// Calculate W: gravity with correction to neutralize time change,
  /// Using this W(t_k) together with Imu() yields the exact dynamics update,
  /// but t stays 0, and hence we stay within NavState sub-group at all times.
  static Gal3 TimeZeroingGravity(const Vector3& g_n, double dt) {
    return {Rot3(), -g_n * (0.5 * dt * dt), g_n * dt, -dt};
  }

  /// Calculate W: position-compensated gravity (left composition) that enables
  /// tracking absolute time in-state. Using this W(t_k) together with Imu()
  /// yields the exact dynamics update with additionally t_{k+1} = t_k + dt.
  static Gal3 CompensatedGravity(const Vector3& g_n, double dt, double t_k) {
    const Point3 pW(-t_k * g_n * dt - g_n * (0.5 * dt * dt));
    const Vector3 vW = g_n * dt;
    return {Rot3(), pW, vW, 0.0};
  }

  /// Calculate U from raw IMU (no gravity): body-frame increments
  static Gal3 Imu(const Vector3& omega_b, const Vector3& f_b, double dt) {
    Gal3::TangentVector xi;
    xi << omega_b, f_b, Z_3x1, 1.0;
    return Gal3::Expmap(xi * dt);
  }

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
   * @param X Current Gal3.
   * @param omega_b Body angular velocity measurement (rad/s).
   * @param f_b Body specific force measurement (m/s^2).
   * @param dt Time step in seconds.
   * @param A Optional Jacobian of the dynamics with respect to the state.
   * @return The next Gal3 after applying the dynamics.
   */
  static Gal3 Dynamics(const Vector3& g_n, const Gal3& X,
                       const Vector3& omega_b, const Vector3& f_b, double dt,
                       Mode mode = TRACK_TIME_WITH_COVARIANCE,
                       OptionalJacobian<10, 10> A = {});

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
  Mode mode_{TRACK_TIME_NO_COVARIANCE};
  Covariance Q_ = Covariance::Zero();
};

}  // namespace gtsam