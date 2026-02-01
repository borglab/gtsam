/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  NavStateImuEKF.cpp
 * @brief Extended Kalman Filter derived class for IMU-driven NavState.
 *
 * @date  August 2025
 * @authors Derek Benham, Frank Dellaert
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/NavStateImuEKF.h>

namespace gtsam {

NavStateImuEKF::NavStateImuEKF(const NavState& X0, const Covariance& P0,
                               const std::shared_ptr<PreintegrationParams>& p)
    : Base(X0, P0), params_(p) {
  // Build process noise Q_ = block_diag(Cg, Ci, Ca)
  Q_.setZero();
  Q_.template block<3, 3>(0, 0) = p->gyroscopeCovariance;
  Q_.template block<3, 3>(3, 3) = p->integrationCovariance;
  Q_.template block<3, 3>(6, 6) = p->accelerometerCovariance;
}

NavState NavStateImuEKF::Imu(const Vector3& omega_b, const Vector3& f_b,
                             double dt) {
  const Vector3 phi_b = omega_b * dt;
  const so3::DexpFunctor local(phi_b);
  const Matrix3 J_l = local.Jacobian().left();
  const Matrix3 N_l = local.Gamma().left();
  return {Rot3::Expmap(phi_b), N_l * f_b * dt * dt, J_l * f_b * dt};
}

NavState NavStateImuEKF::Dynamics(const Vector3& g_n, const NavState& X,
                                  const Vector3& omega_b, const Vector3& f_b,
                                  double dt, OptionalJacobian<9, 9> A) {
  if (dt <= 0.0) {
    throw std::invalid_argument(
        "NavStateImuEKF::Dynamics: dt must be positive");
  }

  // Calculate W, phi, and U
  const NavState W = Gravity(g_n, dt);
  NavState::AutonomousFlow phi{dt};  // Φ: velocity acts on position
  const NavState U = Imu(omega_b, f_b, dt);

  return Base::Dynamics(W, phi, X, U, A);
}

void NavStateImuEKF::predict(const Vector3& omega_b, const Vector3& f_b,
                             double dt) {
  if (dt <= 0.0) {
    throw std::invalid_argument("NavStateImuEKF::predict: dt must be positive");
  }

  // Calculate W, phi, and U
  const NavState W = Gravity(params_->n_gravity, dt);
  NavState::AutonomousFlow phi{dt};  // Φ: velocity acts on position
  const NavState U = Imu(omega_b, f_b, dt);

  // Scale continuous-time process noise to the discrete interval [t, t+dt]
  Covariance Qdt = Q_ * dt;

  // EKF predict
  Base::predict(W, phi, U, Qdt);
}

const std::shared_ptr<PreintegrationParams>& NavStateImuEKF::params() const {
  return params_;
}

const Vector3& NavStateImuEKF::gravity() const { return params_->n_gravity; }

const NavStateImuEKF::Covariance& NavStateImuEKF::processNoise() const {
  return Q_;
}

}  // namespace gtsam