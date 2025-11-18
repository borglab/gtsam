/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testNavStateImuEKF.cpp
 * @brief Unit test for NavStateImuEKF, as well as dynamics used.
 * @date April 26, 2025
 * @authors Scott Baker, Matt Kielo, Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Gal3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/linear/HessianFactor.h>
#include <gtsam/linear/JacobianFactor.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/linear/VectorValues.h>
#include <gtsam/navigation/LeftLinearEKF.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/NavStateImuEKF.h>
#include <gtsam/navigation/PreintegrationParams.h>

using namespace gtsam;

namespace nontrivial_navstate_example {
// A nontrivial NavState
Rot3 R0 = Rot3::RzRyRx(0.1, -0.2, 0.3);
Point3 p0(0.5, -0.4, 0.3);
Vector3 v0(0.2, -0.1, 0.05);
NavState X0(R0, p0, v0);

// Controls and parameters
Vector3 omega_b(0.3, -0.2, 0.1);
Vector3 f_b(0.5, -0.3, 0.2);
auto params = PreintegrationParams::MakeSharedU(9.81);
}  // namespace nontrivial_navstate_example

/* ************************************************************************* */
TEST(NavStateImuEKF, DefaultProcessNoiseFromParams) {
  using namespace nontrivial_navstate_example;

  // GIVEN params with specific covariances
  auto params = PreintegrationParams::MakeSharedU(9.81);
  Matrix3 Cg = (Matrix3() << 0.01, 0, 0, 0, 0.02, 0, 0, 0, 0.03).finished();
  Matrix3 Ci = (Matrix3() << 0.001, 0, 0, 0, 0.002, 0, 0, 0, 0.003).finished();
  Matrix3 Ca = (Matrix3() << 0.1, 0, 0, 0, 0.2, 0, 0, 0, 0.3).finished();
  params->setGyroscopeCovariance(Cg);
  params->setIntegrationCovariance(Ci);
  params->setAccelerometerCovariance(Ca);

  Matrix9 P0 = I_9x9 * 0.01;
  NavStateImuEKF ekf(X0, P0, params);

  Matrix9 Q = Matrix9::Zero();
  Q.block<3, 3>(0, 0) = Cg;
  Q.block<3, 3>(3, 3) = Ci;
  Q.block<3, 3>(6, 6) = Ca;
  EXPECT(assert_equal(Q, ekf.processNoise(), 1e-12));
}

/* ************************************************************************* */
TEST(NavStateImuEKF, DynamicsJacobian) {
  using namespace nontrivial_navstate_example;

  // Check the Jacobian of dynamics
  double dt = 0.01;
  Matrix9 A;
  (void)NavStateImuEKF::Dynamics(params->n_gravity, X0, omega_b, f_b, dt, A);
  std::function<NavState(const NavState&)> f =
      [&](const NavState& Xq) -> NavState {
    return NavStateImuEKF::Dynamics(params->n_gravity, Xq, omega_b, f_b, dt);
  };
  Matrix9 expected = numericalDerivative11(f, X0);

  EXPECT(assert_equal(expected, A, 1e-6));
}

/* ************************************************************************* */
TEST(NavStateImuEKF, PredictMatchesExplicitIntegration) {
  using namespace nontrivial_navstate_example;
  double dt = 10;

  // Explicit integration from paper
  const Vector3 phi_b = omega_b * dt;
  const so3::DexpFunctor local(phi_b);
  const Matrix3 J_l = local.Jacobian().left();
  const Matrix3 N_l = local.Gamma().left();

  const NavState U{Rot3::Expmap(phi_b),  // R_new
                   N_l * f_b * dt * dt,  // p_new
                   J_l * f_b * dt};      // v_new

  // Check against static IMU function:
  const NavState U_static = NavStateImuEKF::Imu(omega_b, f_b, dt);
  EXPECT(assert_equal(U, U_static, 1e-9));

  // Check against Gal3::Expmap
  Vector10 xi;
  xi << omega_b, f_b, Vector3::Zero(), 1.0;
  const Gal3 T = Gal3::Expmap(xi * dt);
  const NavState U_gal3{T.rotation(), T.position(), T.velocity()};
  EXPECT(assert_equal(U_gal3, U, 1e-9));

  // Explicit integration, combined with gravity and v0 * dt boost
  const Vector3& g_n = params->n_gravity;
  const Rot3 R_new = R0 * U.attitude();
  const Vector3 v_new = v0 + g_n * dt + R0 * U.velocity();
  const Point3 p_new = p0 + g_n * (0.5 * dt * dt) + v0 * dt + R0 * U.position();
  const NavState X_explicit(R_new, p_new, v_new);

  // Increment-based integration should match exactly
  auto params = PreintegrationParams::MakeSharedU(9.81);
  NavStateImuEKF ekf(X0, I_9x9 * 1e-3, params);
  NavState X_next = NavStateImuEKF::Dynamics(g_n, X0, omega_b, f_b, dt);
  EXPECT(assert_equal(X_explicit, X_next, 1e-12));
}

/* ***************************************************************************/
// Check Jacobian for world-position measurement h(X)=position(X).
TEST(NavStateImuEKF, PositionMeasurementJacobian) {
  using namespace nontrivial_navstate_example;

  // GIVEN a nontrivial state X0
  const NavState& X = X0;

  // Analytic Jacobian H = dh/d local(X) for h(X)=position(X)
  // With left-invariant chart and NavState compose, dp is in body frame:
  // p' = p + R * dp  => H = [0, R, 0]
  Matrix39 H;
  H.setZero();
  H.block<3, 3>(0, 3) = X.attitude().matrix();

  // Numerical Jacobian via central differencing
  std::function<Point3(const NavState&)> h = [](const NavState& Xq) {
    return Xq.position();
  };
  Matrix39 expected = numericalDerivative11<Point3, NavState>(h, X);

  EXPECT(assert_equal(expected, H, 1e-6));
}

/* ************************************************************************* */
// Sanity-check a single position update using updateWithVector.
// Verifies delta_xi = K * innovation and covariance reduction in pos block.
TEST(NavStateImuEKF, PositionUpdateSanity) {
  using namespace nontrivial_navstate_example;

  // GIVEN an EKF with diagonal covariance (no cross-terms)
  Matrix9 P0 = Matrix9::Zero();
  P0.block<3, 3>(0, 0) = I_3x3 * 1e-3;  // rot
  P0.block<3, 3>(3, 3) = I_3x3 * 1.0;   // pos
  P0.block<3, 3>(6, 6) = I_3x3 * 0.5;   // vel
  auto params = PreintegrationParams::MakeSharedU(9.81);
  NavStateImuEKF ekf(X0, P0, params);

  // BEFORE update: capture state and covariance
  const NavState X_before = ekf.state();
  const Matrix9 P_prior = ekf.covariance();

  // Position measurement: z = p + d
  const Vector3 d(1.0, -2.0, 0.5);
  const Point3 p_true =
      X_before.position() + d;  // pretend ground truth is offset
  const Vector3 z = Vector3(p_true.x(), p_true.y(), p_true.z());

  // Predicted measurement and Jacobian H = [0 R 0]
  const Vector3 prediction =
      Vector3(X_before.position().x(), X_before.position().y(),
              X_before.position().z());
  Matrix39 H;
  H.setZero();
  const Matrix3 Rworld = X_before.attitude().matrix();
  H.block<3, 3>(0, 3) = Rworld;

  // Reasonable measurement noise
  const double sigma = 0.1;  // meters
  Matrix3 Rmeas = I_3x3 * (sigma * sigma);

  // Manually compute K and delta_xi expected
  const Matrix3 S = H * P_prior * H.transpose() + Rmeas;
  const Matrix93 K = P_prior * H.transpose() * S.inverse();
  const Vector3 innovation =
      z - prediction;  // Vector measurement: y = z - prediction
  const Vector9 delta_expected = K * innovation;

  // WHEN performing the EKF update
  ekf.updateWithVector(prediction, H, z, Rmeas);

  // THEN: delta_xi applied equals expected (within tolerance)
  const NavState& X_after = ekf.state();
  const Vector9 delta_ekf = X_before.localCoordinates(X_after);
  EXPECT(assert_equal(delta_expected, delta_ekf, 1e-9));

  // AND: position moved toward z in world frame approximately by R*dp
  const Vector3 dp_body = delta_expected.segment<3>(3);
  const Point3 p_expected = X_before.position() + Point3(Rworld * dp_body);
  EXPECT(assert_equal(p_expected, X_after.position(), 1e-9));

  // AND: covariance position block decreased
  const Matrix9 P_post = ekf.covariance();
  const double trace_pos_prior = P_prior.block<3, 3>(3, 3).trace();
  const double trace_pos_post = P_post.block<3, 3>(3, 3).trace();
  CHECK(trace_pos_post < trace_pos_prior);

  // Compare EKF position update to solving an equivalent GaussianFactorGraph.
  using symbol_shorthand::X;
  const Key key = X(0);
  GaussianFactorGraph gfg;

  // Prior as JacobianFactor using full Gaussian covariance:
  // || I * x - 0||_{P^{-1}}
  Vector b = Vector::Zero(9);
  gfg.add(key, I_9x9, b, noiseModel::Diagonal::Variances(P_prior.diagonal()));

  // Measurement as JacobianFactor using full Gaussian covariance:
  // || H * x -innovation ||_{R^{-1}}
  Vector b_meas = innovation;  // 3x1
  gfg.add(key, H, b_meas, noiseModel::Isotropic::Sigma(3, sigma));

  // Solve for MAP delta
  VectorValues delta_map = gfg.optimize();
  const Vector delta_graph = delta_map.at(key);

  // THEN: Graph solution equals EKF correction
  EXPECT(assert_equal(delta_graph, delta_ekf, 1e-9));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
