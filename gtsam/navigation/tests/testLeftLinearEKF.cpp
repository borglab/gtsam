/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testLeftLinearEKF.cpp
 * @brief Unit tests for the LeftLinearEKF class
 * @date August, 2025
 * @authors Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/LeftLinearEKF.h>
#include <gtsam/navigation/NavState.h>

#include <iostream>

using namespace gtsam;

TEST(LeftLinearEKF, WPhiU_matches_IMU_dynamics_with_gravity) {
  const double dt = 1e-2;  // 10 ms
  const Vector3 n_gravity(0, 0, -9.81);
  const Vector3 gyro(0.01, -0.02, 0.03);   // rad/s (body)
  const Vector3 accel(0.10, -0.05, 0.20);  // m/s^2 (body specific force)

  // Initial state X0 = (R, p, v)
  const Rot3 R0 = Rot3::RzRyRx(0.10, -0.20, 0.05);
  const Point3 p0(1.0, 2.0, 3.0);
  const Vector3 v0(0.5, -0.3, 0.8);
  const NavState X0(R0, p0, v0);

  // Build W (gravity-only left composition, world-frame increments)
  const Point3 pW = 0.5 * n_gravity * dt * dt;
  const Vector3 vW = n_gravity * dt;
  const NavState W(Rot3(), pW, vW);

  // Φ functor: velocity acts on position
  NavState::AutonomousFlow phi{dt};

  // Build U from raw IMU (no gravity): body-frame increments
  const Rot3 dR = Rot3::Expmap(gyro * dt);
  const Vector3 dp_body = accel * (0.5 * dt * dt);
  const Vector3 dv_body = accel * dt;
  const NavState U(dR, dp_body, dv_body);

  // Run LeftLinearEKF predict with Q = 0
  NavState::Jacobian A;
  using EKF = LeftLinearEKF<NavState>;
  NavState Xp = EKF::Dynamics(W, phi, X0, U, A);

  // Closed-form expected result of NavState IMU dynamics with gravity
  const Rot3 R_expected = R0 * dR;
  // v + R a dt + g dt:
  const Vector3 v_expected = v0 + R0.rotate(dv_body) + n_gravity * dt;
  // p + v dt + R(0.5 a dt^2) + 0.5 g dt^2:
  const Point3 p_expected = pW + p0 + v0 * dt + R0.rotate(dp_body);
  const NavState X_expected(R_expected, p_expected, v_expected);
  CHECK(assert_equal(X_expected, Xp));

  // Check A against numerical derivative

  auto numericalA = numericalDerivative11<NavState, NavState>(
      [&](const NavState& X) { return EKF::Dynamics(W, phi, X, U, {}); }, X0);
  CHECK(assert_equal(numericalA, A, 1e-9));

  // Initialize filter
  EKF::Covariance P0 = I_9x9;
  EKF ekf(X0, P0);

  // Run LeftLinearEKF predict with Q = I
  EKF::Covariance Q = Z_9x9;
  ekf.predict(W, phi, U, Q);
  CHECK(assert_equal(X_expected, ekf.state()));
  CHECK(assert_equal<NavState::Jacobian>(A * A.transpose(), ekf.covariance()));
}

int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}