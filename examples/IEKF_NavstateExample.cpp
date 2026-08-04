/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file IEKF_NavstateExample.cpp
 * @brief InvariantEKF on NavState (SE_2(3)) with IMU (predict) and GPS (update)
 * @date April 25, 2025
 * @authors Scott Baker, Matt Kielo, Frank Dellaert
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/base/VectorSpace.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/navigation/InvariantEKF.h>
#include <gtsam/navigation/NavState.h>

#include <iostream>

using namespace std;
using namespace gtsam;

/**
 * @brief Left-invariant dynamics for NavState.
 * @param imu  6×1 vector [a; ω]: linear acceleration and angular velocity.
 * @return     9×1 tangent: [ω; 0₃; a].
 */
Vector9 dynamics(const Vector6& imu) {
  auto a = imu.head<3>();
  auto w = imu.tail<3>();
  Vector9 xi;
  xi << w, Vector3::Zero(), a;
  return xi;
}

/**
 * @brief GPS measurement model: returns position and its Jacobian.
 * @param X    Current state.
 * @param H    Optional 3×9 Jacobian w.r.t. X.
 * @return     3×1 position vector.
 */
Vector3 h_gps(const NavState& X, OptionalJacobian<3, 9> H = {}) {
  return X.position(H);
}

int main() {
  // Initial state & covariances
  NavState X0;  // R=I, v=0, t=0
  Matrix9 P0 = Matrix9::Identity() * 0.1;
  InvariantEKF<NavState> ekf(X0, P0);

  // Noise & timestep
  double dt = 1.0;
  // Continuous-time process noise (scaled by dt inside predict)
  Matrix9 Qc = Matrix9::Identity() * 0.01;
  Matrix3 R = Matrix3::Identity() * 0.5;

  // Two IMU samples [a; ω]
  Vector6 imu1;
  imu1 << 0.1, 0, 0, 0, 0.2, 0;
  Vector6 imu2;
  imu2 << 0, 0.3, 0, 0.4, 0, 0;

  // Two GPS fixes
  Vector3 z1;
  z1 << 0.3, 0, 0;
  Vector3 z2;
  z2 << 0.6, 0, 0;

  cout << "=== Init ===\nX: " << ekf.state() << "\nP: " << ekf.covariance()
    << "\n\n";

  // --- first predict/update ---
  ekf.predict(dynamics(imu1), dt, Qc);
  cout << "--- After predict 1 ---\nX: " << ekf.state()
    << "\nP: " << ekf.covariance() << "\n\n";
  ekf.update(h_gps, z1, R);
  cout << "--- After update 1 ---\nX: " << ekf.state()
    << "\nP: " << ekf.covariance() << "\n\n";

  // --- second predict/update ---
  ekf.predict(dynamics(imu2), dt, Qc);
  cout << "--- After predict 2 ---\nX: " << ekf.state()
    << "\nP: " << ekf.covariance() << "\n\n";
  ekf.update(h_gps, z2, R);
  cout << "--- After update 2 ---\nX: " << ekf.state()
    << "\nP: " << ekf.covariance() << "\n";

  return 0;
}
