/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file IEKF_Rot3Example.cpp
 * @brief Left‐Invariant EKF on SO(3) with state‐dependent pitch/roll control
 * and a single magnetometer update.
 * @date April 25, 2025
 * @authors Scott Baker, Matt Kielo, Frank Dellaert
 */

#include <gtsam/base/Matrix.h>
#include <gtsam/base/OptionalJacobian.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/LieGroupEKF.h>

#include <iostream>

using namespace std;
using namespace gtsam;

// --- 1) Closed‐loop dynamics f(X): xi = –k·[φx,φy,0], H = ∂xi/∂φ·Dφ ---
static constexpr double k = 0.5;
Vector3 dynamicsSO3(const Rot3& X, OptionalJacobian<3, 3> H = {}) {
  // φ = Logmap(R), Dφ = ∂φ/∂δR
  Matrix3 D_phi;
  Vector3 phi = Rot3::Logmap(X, D_phi);
  // zero out yaw
  phi[2] = 0.0;
  D_phi.row(2).setZero();

  if (H) *H = -k * D_phi;  // ∂(–kφ)/∂δR
  return -k * phi;         // xi ∈ 𝔰𝔬(3)
}

// --- 2) Magnetometer model: z = R⁻¹ m, H = –[z]_× ---
static const Vector3 m_world(0, 0, -1);
Vector3 h_mag(const Rot3& X, OptionalJacobian<3, 3> H = {}) {
  Vector3 z = X.inverse().rotate(m_world);
  if (H) *H = -skewSymmetric(z);
  return z;
}

int main() {
  // Initial estimate (identity) and covariance
  const Rot3 R0 = Rot3::RzRyRx(0.1, -0.2, 0.3);
  const Matrix3 P0 = Matrix3::Identity() * 0.1;
  LieGroupEKF<Rot3> ekf(R0, P0);

  // Timestep, process noise, measurement noise
  double dt = 0.1;
  Matrix3 Q = Matrix3::Identity() * 0.01;
  Matrix3 Rm = Matrix3::Identity() * 0.05;

  cout << "=== Init ===\nR:\n"
       << ekf.state().matrix() << "\nP:\n"
       << ekf.covariance() << "\n\n";

  // Predict using state‐dependent f
  ekf.predict(dynamicsSO3, dt, Q);
  cout << "--- After predict ---\nR:\n" << ekf.state().matrix() << "\n\n";

  // Magnetometer measurement = body‐frame reading of m_world
  Vector3 z = h_mag(R0);
  ekf.update(h_mag, z, Rm);
  cout << "--- After update ---\nR:\n" << ekf.state().matrix() << "\n";

  return 0;
}
