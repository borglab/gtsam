/**
 * @file  testMEKFDynamics.cpp
 * @brief Unit tests for strapdown propagation + finite-difference Jacobian F.
 *
 * Tangent block order: R, v, p, b_gyro, b_accel.
 */
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam_unstable/navigation/MEKF.h>

using namespace gtsam::mekf;
using namespace gtsam;

static constexpr double kTol = 1e-9;

using Tangent = Eigen::Matrix<double, 15, 1>;
using Mat15 = Eigen::Matrix<double, 15, 15>;

static bool veq(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                double tol = kTol) {
  return assert_equal((Vector)a, (Vector)b, tol);
}
static bool meq(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b,
                double tol = kTol) {
  return assert_equal((Matrix)a, (Matrix)b, tol);
}

// The dynamics are frame-agnostic -- gravity arrives in ImuInput::g_vec -- so
// these tests pick a convention of their own: Z-up (ENU), gravity along -z.
static Eigen::Vector3d testGravity() {
  return Eigen::Vector3d(0.0, 0.0, -9.81);
}

// Specific force that cancels gravity for a level vehicle (R = I): f = -g.
static Eigen::Vector3d hoverAccel() { return -testGravity(); }  // (0, 0, 9.81)

/// A zeroed input carrying the fixture's gravity.
static ImuInput gravityInput() {
  ImuInput u;
  u.g_vec = testGravity();
  return u;
}

// ---------------------------------------------------------------------------
// gravity
// ---------------------------------------------------------------------------

// Gravity comes from the input, not a baked-in constant: flipping g_vec flips
// the velocity increment, which is what makes the filter frame-agnostic.
TEST(MEKFDynamics, PropagateUsesInputGravity) {
  const double dt = 0.1;
  const MEKFState X = MEKFState::identity();
  ImuInput up = gravityInput();  // g = (0, 0, -9.81)
  ImuInput un = gravityInput();
  un.g_vec = -testGravity();  // g = (0, 0, +9.81), i.e. a Z-down frame
  EXPECT(veq(propagateMean(X, up, dt).v, testGravity() * dt));
  EXPECT(veq(propagateMean(X, un, dt).v, -testGravity() * dt));
}

// ---------------------------------------------------------------------------
// propagateMean
// ---------------------------------------------------------------------------

// Level hover: gravity cancelled, no rotation -> state held (only p tracks v).
TEST(MEKFDynamics, PropagateHoverHoldsState) {
  MEKFState X = MEKFState::identity();
  ImuInput u = gravityInput();
  u.omega = Eigen::Vector3d::Zero();
  u.accel = hoverAccel();
  MEKFState Xn = propagateMean(X, u, 0.1);
  EXPECT(Xn.R.equals(Rot3::Identity(), kTol));
  EXPECT(veq(Xn.v, Eigen::Vector3d::Zero()));
  EXPECT(veq(Xn.p, Eigen::Vector3d::Zero()));
}

// Constant velocity: position integrates v (Euler: p+ = p + v dt).
TEST(MEKFDynamics, PropagateConstantVelocityIntegratesPosition) {
  const double dt = 0.1;
  MEKFState X(Rot3::Identity(), Eigen::Vector3d(1, 0, 0),
              Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
              Eigen::Vector3d::Zero());
  ImuInput u = gravityInput();
  u.accel = hoverAccel();  // cancel gravity -> zero linear acceleration
  MEKFState Xn = propagateMean(X, u, dt);
  EXPECT(veq(Xn.v, Eigen::Vector3d(1, 0, 0)));
  EXPECT(veq(Xn.p, Eigen::Vector3d(0.1, 0, 0)));
}

// Attitude integrates the bias-corrected gyro: R+ = R Exp((omega - b_gyro) dt).
TEST(MEKFDynamics, PropagateAttitudeUsesGyroBias) {
  const double dt = 0.1;
  const Eigen::Vector3d bg(0, 0, 0.2);
  MEKFState X(Rot3::Rz(0.3), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
              bg, Eigen::Vector3d::Zero());
  ImuInput u = gravityInput();
  u.omega = Eigen::Vector3d(0, 0, 0.5);
  u.accel = hoverAccel();
  MEKFState Xn = propagateMean(X, u, dt);
  const Eigen::Vector3d expected_corr(0, 0, 0.3);  // 0.5 - 0.2
  EXPECT(Xn.R.equals(X.R * Rot3::Expmap(expected_corr * dt), kTol));
}

// Velocity integrates the bias-corrected, rotated specific force + gravity.
TEST(MEKFDynamics, PropagateVelocityUsesAccelBias) {
  const double dt = 0.1;
  const Eigen::Vector3d ba(0, 0, 1.0);
  MEKFState X(Rot3::Identity(), Eigen::Vector3d::Zero(),
              Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), ba);
  ImuInput u = gravityInput();
  u.accel = hoverAccel();  // (0,0,9.81); corrected = (0,0,8.81)
  MEKFState Xn = propagateMean(X, u, dt);
  // a_n = R(accel - ba) + g = (0,0,8.81) + (0,0,-9.81) = (0,0,-1)
  EXPECT(veq(Xn.v, Eigen::Vector3d(0, 0, -0.1)));
}

// Biases are constant through propagation.
TEST(MEKFDynamics, PropagateBiasesUnchanged) {
  const Eigen::Vector3d bg(0.01, -0.02, 0.03), ba(-0.1, 0.2, 0.0);
  MEKFState X(Rot3::Ry(0.2), Eigen::Vector3d(1, 2, 3), Eigen::Vector3d(4, 5, 6),
              bg, ba);
  ImuInput u = gravityInput();
  u.omega = Eigen::Vector3d(0.1, 0.2, 0.3);
  u.accel = Eigen::Vector3d(0.5, 0.0, 9.0);
  MEKFState Xn = propagateMean(X, u, 0.05);
  EXPECT(veq(Xn.b_gyro, bg));
  EXPECT(veq(Xn.b_accel, ba));
}

// ---------------------------------------------------------------------------
// transitionJacobian
// ---------------------------------------------------------------------------

// Build a representative excited state + input for Jacobian tests.
static MEKFState exampleState() {
  return MEKFState(
      Rot3::Rz(0.4) * Rot3::Rx(0.2), Eigen::Vector3d(1.0, -0.5, 0.3),
      Eigen::Vector3d(2.0, 1.0, -1.0), Eigen::Vector3d(0.01, -0.02, 0.03),
      Eigen::Vector3d(-0.05, 0.1, 0.0));
}
static ImuInput exampleInput() {
  ImuInput u = gravityInput();
  u.omega = Eigen::Vector3d(0.2, -0.1, 0.3);
  u.accel = Eigen::Vector3d(0.5, 0.2, 9.5);
  return u;
}

// First-order consistency: for a small perturbation delta,
//   Local(Xn_ref, propagateMean(Retract(X, delta))) ~ F * delta
// to within O(||delta||^2).
TEST(MEKFDynamics, JacobianFirstOrderConsistency) {
  const double dt = 0.01;
  const MEKFState X = exampleState();
  const ImuInput u = exampleInput();
  const Mat15 F = transitionJacobian(X, u, dt);
  const MEKFState Xn_ref = propagateMean(X, u, dt);

  // A few independent small directions (norm ~1e-4 -> O(1e-8) truncation).
  for (int trial = 0; trial < 5; ++trial) {
    Tangent delta = Tangent::Zero();
    for (int k = 0; k < 15; ++k) {
      delta(k) = std::sin(1.0 + trial + 0.37 * k);
    }
    delta *= 1e-4;

    const MEKFState Xp = traits<MEKFState>::Retract(X, delta);
    const Tangent actual =
        traits<MEKFState>::Local(Xn_ref, propagateMean(Xp, u, dt));
    const Tangent predicted = F * delta;
    EXPECT(assert_equal((Vector)actual, (Vector)predicted, 1e-9));
  }
}

// Known-exact blocks of F (independent of finite-difference noise):
//   position row: d(dp+)/d(dp) = I,   d(dp+)/d(dv) = I*dt
//   bias rows are identity (biases are static, decoupled).
TEST(MEKFDynamics, JacobianKnownBlocks) {
  const double dt = 0.02;
  const MEKFState X = exampleState();
  const ImuInput u = exampleInput();
  const Mat15 F = transitionJacobian(X, u, dt);
  const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();

  EXPECT(meq(F.block<3, 3>(6, 6), I3, 1e-6));       // dp <- dp
  EXPECT(meq(F.block<3, 3>(6, 3), I3 * dt, 1e-6));  // dp <- dv
  EXPECT(meq(F.block<3, 3>(9, 9), I3, 1e-6));       // d_bg <- d_bg
  EXPECT(meq(F.block<3, 3>(12, 12), I3, 1e-6));     // d_ba <- d_ba
  // Bias rows decouple from everything else.
  EXPECT(meq(F.block<3, 6>(9, 0), Eigen::Matrix<double, 3, 6>::Zero(), 1e-6));
  EXPECT(meq(F.block<3, 3>(9, 12), Eigen::Matrix3d::Zero(), 1e-6));
  EXPECT(meq(F.block<3, 9>(12, 0), Eigen::Matrix<double, 3, 9>::Zero(), 1e-6));
}

// Velocity-from-accel-bias block is -R*dt -> state dependent (varies with R).
// This is the hallmark MEKF property: F depends on the current estimate.
TEST(MEKFDynamics, JacobianStateDependent) {
  const double dt = 0.02;
  const ImuInput u = exampleInput();

  MEKFState X1 = exampleState();
  MEKFState X2 = exampleState();
  X2.R = Rot3::Ry(1.1) * X2.R;  // different attitude only

  const Mat15 F1 = transitionJacobian(X1, u, dt);
  const Mat15 F2 = transitionJacobian(X2, u, dt);

  // dv <- d_ba equals -R*dt; differs because R differs.
  EXPECT(meq(F1.block<3, 3>(3, 12), -X1.R.matrix() * dt, 1e-6));
  EXPECT(meq(F2.block<3, 3>(3, 12), -X2.R.matrix() * dt, 1e-6));
  EXPECT((F1.block<3, 3>(3, 12) - F2.block<3, 3>(3, 12)).norm() > 1e-3);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
