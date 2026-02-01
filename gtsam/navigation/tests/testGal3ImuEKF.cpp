/* ----------------------------------------------------------------------------
 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
 *
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file testGal3ImuEKF.cpp
 * @brief Unit test for Gal3ImuEKF, as well as dynamics used.
 * @date September 2025
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
#include <gtsam/navigation/Gal3ImuEKF.h>
#include <gtsam/navigation/LeftLinearEKF.h>
#include <gtsam/navigation/PreintegrationParams.h>
#include <gtsam/navigation/Scenario.h>
#include <gtsam/navigation/ScenarioRunner.h>

#include <iostream>

using namespace gtsam;

namespace nontrivial_gal3_example {

// A nontrivial Gal3
using Matrix10 = Eigen::Matrix<double, 10, 10>;
using Matrix310 = Eigen::Matrix<double, 3, 10>;
using Matrix10x3 = Eigen::Matrix<double, 10, 3>;
static const Matrix10 I_10x10 = Matrix10::Identity();
Rot3 R0 = Rot3::RzRyRx(0.1, -0.2, 0.3);
Point3 p0(0.5, -0.4, 0.3);
Vector3 v0(0.2, -0.1, 0.05);
double t0 = 10.0;
Gal3 X0(R0, p0, v0, t0);

// Controls and parameters
Vector3 omega_b(0.3, -0.2, 0.1);
Vector3 f_b(0.5, -0.3, 0.2);
auto params = PreintegrationParams::MakeSharedU(9.81);
}  // namespace nontrivial_gal3_example

/* ************************************************************************* */
TEST(Gal3ImuEKF, DefaultProcessNoiseFromParams) {
  using namespace nontrivial_gal3_example;

  // GIVEN params with specific covariances
  auto params = PreintegrationParams::MakeSharedU(9.81);
  Matrix3 Cg = (Matrix3() << 0.01, 0, 0, 0, 0.02, 0, 0, 0, 0.03).finished();
  Matrix3 Ci = (Matrix3() << 0.001, 0, 0, 0, 0.002, 0, 0, 0, 0.003).finished();
  Matrix3 Ca = (Matrix3() << 0.1, 0, 0, 0, 0.2, 0, 0, 0, 0.3).finished();
  params->setGyroscopeCovariance(Cg);
  params->setIntegrationCovariance(Ci);
  params->setAccelerometerCovariance(Ca);

  Matrix10 P0 = I_10x10 * 0.01;
  Gal3ImuEKF ekf(X0, P0, params);

  Matrix10 Q = Matrix10::Zero();
  Q.block<3, 3>(0, 0) = Cg;
  Q.block<3, 3>(6, 6) = Ci;
  Q.block<3, 3>(3, 3) = Ca;
  EXPECT(assert_equal(Q, ekf.processNoise(), 1e-12));
}

/* ************************************************************************* */
// Check the Jacobian of dynamics, for all three modes.
TEST(Gal3ImuEKF, DynamicsJacobian) {
  using namespace nontrivial_gal3_example;
  Matrix10 A;
  double dt = 0.01;
  const Vector3& n_g = params->n_gravity;
  // Gal3ImuEKF::TRACK_TIME_NO_COVARIANCE is not expected to pass.
  for (auto mode :
       {Gal3ImuEKF::NO_TIME, Gal3ImuEKF::TRACK_TIME_WITH_COVARIANCE}) {
    (void)Gal3ImuEKF::Dynamics(n_g, X0, omega_b, f_b, dt, mode, A);
    std::function<Gal3(const Gal3&)> f = [&](const Gal3& Xq) -> Gal3 {
      return Gal3ImuEKF::Dynamics(n_g, Xq, omega_b, f_b, dt, mode);
    };
    Matrix10 expected = numericalDerivative11(f, X0);
    EXPECT(assert_equal(expected, A, 1e-6));
  }
}

/* ************************************************************************* */
TEST(Gal3ImuEKF, PredictMatchesExplicitIntegration) {
  using namespace nontrivial_gal3_example;
  double dt = 10;

  // Explicit integration from paper
  const Vector3 phi_b = omega_b * dt;
  const so3::DexpFunctor local(phi_b);
  const Matrix3 J_l = local.Jacobian().left();
  const Matrix3 N_l = local.Gamma().left();

  const Gal3 U{Rot3::Expmap(phi_b), N_l * f_b * dt * dt, J_l * f_b * dt, dt};

  // Check against static IMU function:
  const Gal3 U_static = Gal3ImuEKF::Imu(omega_b, f_b, dt);
  EXPECT(assert_equal(U, U_static, 1e-9));

  // Check against Gal3::Expmap
  Vector10 xi;
  xi << omega_b, f_b, Vector3::Zero(), 1.0;
  const Gal3 U_gal3 = Gal3::Expmap(xi * dt);
  EXPECT(assert_equal(U_gal3, U, 1e-9));

  // Explicit integration, combined with gravity and v0 * dt boost
  const Vector3& g_n = params->n_gravity;
  const Rot3 R_new = R0 * U.attitude();
  const Vector3 v_new = v0 + g_n * dt + R0 * U.velocity();
  const Point3 p_new = p0 + g_n * (0.5 * dt * dt) + v0 * dt + R0 * U.position();
  const Gal3 X_explicit(R_new, p_new, v_new, t0 + dt);

  // Increment-based integration should match exactly
  auto params = PreintegrationParams::MakeSharedU(9.81);
  Gal3ImuEKF ekf(X0, I_10x10 * 1e-3, params);
  Gal3 X_next = Gal3ImuEKF::Dynamics(g_n, X0, omega_b, f_b, dt);
  EXPECT(assert_equal(X_explicit, X_next, 1e-12));
}

/* ************************************************************************* */
// Test predict with a scenario that can be integrated exactly.
TEST(Gal3ImuEKF, PredictMatchesScenario) {
  // --- Scenario: camera orbiting a fixed point ---
  // measurements in the body/sensor frame, assumed FRD
  double radius = 30.0;
  double angular_velocity = M_PI;       // rad/sec (half-turn per second)
  Vector3 w_b(0, 0, angular_velocity);  // body yaw rate
  Vector3 v_n(radius * angular_velocity, 0, 0);  // world-frame velocity
  ConstantTwistScenario scenario(w_b, v_n);

  // using NED coordinates as the navigation frame
  auto params = PreintegrationParams::MakeSharedD(10);  // use 10 m/s^2 gravity

  // EKF setup with initial state and covariance, FRD frame aligned with NED
  // (i.e., looking north)
  const Gal3 X0 = scenario.gal3(0.0);          // Get state at t=0
  Matrix P0 = Matrix::Identity(10, 10) * 0.1;  // std ~ 0.316 on each component
  P0(9, 9) = 1e-6;                             // small variance for time

  ScenarioRunner runner(scenario, params, 1.0);
  Gal3ImuEKF ekf(X0, P0, params);

  // Predict every 0.5 second for 2 seconds
  double T = 2.0, dt = 0.5;
  size_t N = T / dt;
  Vector3 omega_b = runner.actualAngularVelocity(0.0);
  Vector3 f_b = runner.actualSpecificForce(0.0);
  for (size_t i = 0; i < N; i++) {
    ekf.predict(omega_b, f_b, dt);
    // Check that predicted state matches ground truth
    double t = dt * (i + 1);
    EXPECT(assert_equal(scenario.gal3(t), ekf.state(), 1e-9));
  }
}

/* ************************************************************************* */
// Check Jacobian for world-position measurement h(X)=position(X).
TEST(Gal3ImuEKF, PositionMeasurementJacobian) {
  using namespace nontrivial_gal3_example;

  // GIVEN a nontrivial state X0
  const Gal3& X = X0;

  // Analytic Jacobian H = dh/d local(X) for h(X)=position(X)
  // With left-invariant chart and NavState compose, dp is in body frame:
  // p' = p + R * dp  => H = [0, R, 0]
  Matrix310 H;
  H.setZero();
  H.block<3, 3>(0, 6) = X.attitude().matrix();
  H.block<3, 1>(0, 9) = X.velocity();

  // Numerical Jacobian via central differencing
  std::function<Point3(const Gal3&)> h = [](const Gal3& Xq) {
    return Xq.position();
  };
  Matrix310 expected = numericalDerivative11<Point3, Gal3>(h, X);

  EXPECT(assert_equal(expected, H, 1e-6));
}
/* ************************************************************************* */
// Sanity-check a single position update using updateWithVector. Verifies
// delta_xi = K * innovation and covariance reduction in pos block.
TEST(Gal3ImuEKF, PositionUpdateSanity) {
  using namespace nontrivial_gal3_example;

  // GIVEN an EKF with diagonal covariance (no cross-terms)
  Matrix10 P0 = Matrix10::Zero();
  P0.block<3, 3>(0, 0) = I_3x3 * 1e-3;  // rot
  P0.block<3, 3>(3, 3) = I_3x3 * 0.5;   // vel
  P0.block<3, 3>(6, 6) = I_3x3 * 1.0;   // pos
  P0(9, 9) = 1e-6;
  auto params = PreintegrationParams::MakeSharedU(9.81);
  Gal3ImuEKF ekf(X0, P0, params);

  // BEFORE update: capture state and covariance
  const Gal3 X_before = ekf.state();
  const Matrix10 P_prior = ekf.covariance();

  // Position measurement: z = p + d
  const Vector3 d(1.0, -2.0, 0.5);
  const Point3 p_true =
      X_before.position() + d;  // pretend ground truth is offset
  const Vector3 z = Vector3(p_true.x(), p_true.y(), p_true.z());

  // Predicted measurement and Jacobian H = [0 R 0]
  const Vector3 prediction =
      Vector3(X_before.position().x(), X_before.position().y(),
              X_before.position().z());
  Matrix310 H;
  H.setZero();
  const Matrix3 Rworld = X_before.attitude().matrix();
  H.block<3, 3>(0, 6) = Rworld;
  H.block<3, 1>(0, 9) = X_before.velocity();

  // Reasonable measurement noise
  const double sigma = 0.1;  // meters
  Matrix3 Rmeas = I_3x3 * (sigma * sigma);

  // Manually compute K and delta_xi expected
  const Matrix3 S = H * P_prior * H.transpose() + Rmeas;
  const Matrix10x3 K = P_prior * H.transpose() * S.inverse();
  const Vector3 innovation =
      z - prediction;  // Vector measurement: y = z - prediction
  const Vector10 delta_expected = K * innovation;

  // WHEN performing the EKF update
  ekf.updateWithVector(prediction, H, z, Rmeas);

  // THEN: delta_xi applied equals expected (within tolerance)
  const Gal3& X_after = ekf.state();
  const Vector10 delta_ekf = X_before.localCoordinates(X_after);
  EXPECT(assert_equal(delta_expected, delta_ekf, 1e-9));
  //
  // AND: position moved toward z in world frame approximately by R*dp
  const Vector3 dp_body = delta_expected.segment<3>(6);
  const double dt_correction = delta_expected(9);
  const Point3 p_expected = X_before.position() + Point3(Rworld * dp_body) +
                            X_before.velocity() * dt_correction;
  // const Point3 p_expected = X_before.position() + Point3(Rworld * dp_body);

  EXPECT(assert_equal(p_expected, X_after.position(), 1e-9));

  // // AND: covariance position block decreased
  const Matrix10 P_post = ekf.covariance();
  const double trace_pos_prior = P_prior.block<3, 3>(6, 6).trace();
  const double trace_pos_post = P_post.block<3, 3>(6, 6).trace();
  CHECK(trace_pos_post < trace_pos_prior);
  //
  // Compare EKF position update to solving an equivalent GaussianFactorGraph.
  using symbol_shorthand::X;
  const Key key = X(0);
  GaussianFactorGraph gfg;

  // Prior as JacobianFactor using full Gaussian covariance:
  // || I * x - 0||_{P^{-1}}
  Vector b = Vector::Zero(10);
  gfg.add(key, I_10x10, b, noiseModel::Diagonal::Variances(P_prior.diagonal()));

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
// Verify dynamics W and U.
TEST(Gal3ImuEKF, PredictWithWandU) {
  using namespace nontrivial_gal3_example;

  double dt = 0.1;

  const Gal3 W = Gal3ImuEKF::CompensatedGravity(params->n_gravity, dt, t0);
  const Gal3 U = Gal3ImuEKF::Imu(omega_b, f_b, dt);

  // Compute dynamics
  Matrix10 A_ekf;
  Gal3 X_predicted = Gal3ImuEKF::Base::Dynamics(W, X0, U, A_ekf);

  // New state: W*X0*U
  Gal3 X_expected = W * X0 * U;
  EXPECT(assert_equal(X_expected, X_predicted, 1e-12));

  // Check times
  EXPECT_DOUBLES_EQUAL(t0, X0.time(), 1e-12);
  EXPECT_DOUBLES_EQUAL(t0 + dt, X_predicted.time(), 1e-12);

  // Expected: J = Ad_U^(-1)
  Matrix10 A_expected = U.inverse().AdjointMap();
  EXPECT(assert_equal(A_expected, A_ekf, 1e-9));
}

/* ************************************************************************* */
// Ensure W, U, X match the T_j = Gamma_ij * T_i * Upsilon_ij
TEST(Gal3ImuEKF, ComponentsMatchGamma) {
  using namespace nontrivial_gal3_example;

  // Nontrivial variables
  const double dt = 0.01;
  const Vector3& g = params->n_gravity;

  // Create X, W, U
  const Gal3& X = X0;  // A state snapshot
  const Gal3 W = Gal3ImuEKF::TimeZeroingGravity(g, dt);
  const Gal3 U = Gal3ImuEKF::Imu(omega_b, f_b, dt);

  // 1. Check state time
  EXPECT_DOUBLES_EQUAL(t0, X.time(), 1e-12);

  // 2. Check W
  EXPECT(assert_equal(Rot3(), W.attitude(), 1e-9));
  EXPECT(assert_equal(Point3(-0.5 * g * dt * dt), W.position(), 1e-9));
  EXPECT(assert_equal(Vector3(g * dt), W.velocity(), 1e-9));
  EXPECT_DOUBLES_EQUAL(-dt, W.time(), 1e-12);

  // 3. Check U
  EXPECT(assert_equal(Rot3::Expmap(omega_b * dt), U.attitude(), 1e-9));
  // Frank: Commenting out these two tests, as they are not expected to hold
  // exactly EXPECT(assert_equal(Point3(0.5 * f_b * dt * dt), U.position(),
  // 1e-9)); EXPECT(assert_equal(Vector3(f_b * dt), U.velocity(), 1e-9));
  EXPECT_DOUBLES_EQUAL(dt, U.time(), 1e-12);
}

/* ************************************************************************* */
// Verify W, U match the Exp (G,N) function
TEST(Gal3ImuEKF, FormulationsMatchMatrixExponential) {
  using namespace nontrivial_gal3_example;

  const double dt = 10;
  const Vector3& g = params->n_gravity;

  // Create tangent vectors for G, N,

  // W Matrix W
  Gal3::TangentVector xiG = Gal3::TangentVector::Zero();
  xiG.segment<3>(3) = g;

  // N is given by a dt contribution
  Gal3::TangentVector xiN = Gal3::TangentVector::Zero();
  xiN(9) = 1.0;

  // Compare W
  Gal3 W_expected = Gal3::Expmap((xiG - xiN) * dt);
  Gal3 W_actual = Gal3ImuEKF::TimeZeroingGravity(g, dt);
  EXPECT(assert_equal(W_expected, W_actual, 1e-9));

  // Compare U
  // U is given by exp((w^ - b^ + N)*dt)
  Vector3 rho(0.0, 0.0, 0.0);
  Gal3::TangentVector w;
  w << omega_b, f_b, rho, 0;

  Gal3 U_expected = Gal3::Expmap((w + xiN) * dt);
  Gal3 U_actual = Gal3ImuEKF::Imu(omega_b, f_b, dt);
  EXPECT(assert_equal(U_expected, U_actual, 1e-5));
}

/* ************************************************************************* */
// Check that TimeZeroingGravity and CompensatedGravity match the "correction:
// math" formulas from the paper.
TEST(Gal3ImuEKF, TestCorrectionFormulas) {
  using namespace nontrivial_gal3_example;
  const Vector3 g_n = params->n_gravity;
  double dt = 0.1;
  double t_k = 5.0;

  // Check that TimeZeroingGravity matches the math
  auto timeZeroingGravity = [g_n](double dt) -> Gal3 {
    const Gal3 G = Gal3ImuEKF::Gravity(g_n, dt);
    const Gal3 C{Rot3(), Z_3x1, Z_3x1, -dt};
    return G * C;
  };
  Gal3 W_math = timeZeroingGravity(dt);
  Gal3 W_actual = Gal3ImuEKF::TimeZeroingGravity(g_n, dt);
  EXPECT(assert_equal(W_math, W_actual, 1e-9));

  // Check that CompensatedGravity matches the math
  auto compensatedGravity = [g_n](double dt, double t_k) -> Gal3 {
    const Gal3 G = Gal3ImuEKF::Gravity(g_n, dt);
    const Gal3 C{Rot3(), -t_k * G.velocity() - 2.0 * G.translation(), Z_3x1, 0};
    return G * C;
  };
  Gal3 W_comp_math = compensatedGravity(dt, t_k);
  Gal3 W_comp_actual = Gal3ImuEKF::CompensatedGravity(g_n, dt, t_k);
  EXPECT(assert_equal(W_comp_math, W_comp_actual, 1e-9));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}

/* ************************************************************************* */
