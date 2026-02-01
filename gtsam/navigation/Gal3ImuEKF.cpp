/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file  Gal3ImuEKF.cpp
 * @brief Extended Kalman Filter derived class for IMU-driven Gal3.
 *
 * @date  September 2025
 * @authors Scott Baker
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/Gal3ImuEKF.h>

namespace gtsam {

Gal3ImuEKF::Gal3ImuEKF(const Gal3& X0, const Covariance& P0,
                       const std::shared_ptr<PreintegrationParams>& p,
                       Mode mode)
    : Base(X0, P0), params_(p), mode_(mode) {
  // Build process noise Q_ = block_diag(Cg, Ca, Ci, 0)
  Q_.setZero();
  Q_.template block<3, 3>(0, 0) = p->gyroscopeCovariance;
  Q_.template block<3, 3>(3, 3) = p->accelerometerCovariance;
  Q_.template block<3, 3>(6, 6) = p->integrationCovariance;
}

Gal3 Gal3ImuEKF::Dynamics(const Vector3& g_n, const Gal3& X,
                          const Vector3& omega_b, const Vector3& f_b, double dt,
                          Mode mode, OptionalJacobian<10, 10> A) {
  if (dt <= 0.0) {
    throw std::invalid_argument("Gal3ImuEKF::Dynamics: dt must be positive");
  }

  // Calculate W, phi, and U
  const Gal3 W = (mode == NO_TIME) ? TimeZeroingGravity(g_n, dt)
                                   : CompensatedGravity(g_n, dt, X.time());
  const Gal3 U = Imu(omega_b, f_b, dt);

  const Gal3 X_next = Base::Dynamics(W, X, U, A);
  if (A && mode == TRACK_TIME_WITH_COVARIANCE) {
    // Extra column from state-dependent left factor W(t_k):
    // right-trivialized increment at W due to δt is e_t := [0; 0; -g dt; 0] in
    // the navigation frame
    Vector e_t(10);
    e_t.setZero();
    e_t.segment<3>(6) = -g_n * dt;  // p-block (indices 6..8)

    // Bring to the right-side through the adjoint of X_next and add on to A:
    A->col(9) += X_next.inverse().Adjoint(e_t);
  }
  return X_next;
}

void Gal3ImuEKF::predict(const Vector3& omega_b, const Vector3& f_b,
                         double dt) {
  if (dt <= 0.0) {
    throw std::invalid_argument("Gal3ImuEKF::predict: dt must be positive");
  }

  // Calculate next state, with covariance
  Gal3::Jacobian A;
  X_ = Dynamics(params_->n_gravity, X_, omega_b, f_b, dt, mode_, A);

  // Scale continuous-time process noise to the discrete interval [t, t+dt]
  Covariance Qdt = Q_ * dt;

  // Update covariance
  P_ = A * P_ * A.transpose() + Qdt;
}

const std::shared_ptr<PreintegrationParams>& Gal3ImuEKF::params() const {
  return params_;
}

const Vector3& Gal3ImuEKF::gravity() const { return params_->n_gravity; }

const Gal3ImuEKF::Covariance& Gal3ImuEKF::processNoise() const { return Q_; }

}  // namespace gtsam
