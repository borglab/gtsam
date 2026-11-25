/**
 * @file  testTFGPositionOutput.cpp
 * @brief Unit tests for the direct global position output.
 */
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam_unstable/navigation/TFGInEKF.h>  // PositionOutput, phi

using namespace tfg;
using namespace gtsam;

static constexpr double kTol = 1e-9;
static constexpr double kTolL = 1e-6;

static bool veq(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                double tol = kTol) {
  return assert_equal((Vector)a, (Vector)b, tol);
}
static bool meq(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b,
                double tol = kTol) {
  return assert_equal((Matrix)a, (Matrix)b, tol);
}

static TwoFrameGroup makeState() {
  return TwoFrameGroup::FromState(
      Rot3::Rz(0.4) * Rot3::Rx(0.2), Eigen::Vector3d(0.3, 0.1, -0.4),
      Eigen::Vector3d(1.0, -2.0, 0.5), Eigen::Vector3d(0.02, -0.01, 0.03),
      Eigen::Vector3d(-0.04, 0.05, 0.0));
}

// ---------------------------------------------------------------------------
// predict:  h(xi) = R^T (pi - p)   (Eq. 34)
// ---------------------------------------------------------------------------

TEST(PositionOutput, PredictValue) {
  auto xi = makeState();
  Eigen::Vector3d pi(2.0, -1.0, 1.5);
  Eigen::Vector3d expected = xi.R.unrotate(pi - xi.p);
  EXPECT(veq(PositionOutput::predict(xi, pi), expected));
}

// Noise-free value is zero when the measurement equals the position estimate.
TEST(PositionOutput, PredictZeroWhenConsistent) {
  auto xi = makeState();
  EXPECT(
      veq(PositionOutput::predict(xi, xi.p), Eigen::Vector3d::Zero(), kTolL));
}

// ---------------------------------------------------------------------------
// output_action + equivariance:  h(xi * X) == rho_X(h(xi))   (Eq. 35)
// ---------------------------------------------------------------------------

TEST(PositionOutput, OutputEquivariance) {
  auto xi = makeState();
  auto X = TwoFrameGroup::FromState(
      Rot3::Ry(0.3) * Rot3::Rz(-0.1), Eigen::Vector3d(0.2, -0.3, 0.1),
      Eigen::Vector3d(-0.5, 1.0, 2.0), Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 0.0, 0.0));
  Eigen::Vector3d pi(2.0, -1.0, 1.5);

  Eigen::Vector3d lhs = PositionOutput::predict(phi(X, xi), pi);
  Eigen::Vector3d rhs =
      PositionOutput::output_action(X, PositionOutput::predict(xi, pi));
  EXPECT(veq(lhs, rhs, kTolL));
}

// ---------------------------------------------------------------------------
// jacobian C0: structure + finite-difference correctness
// ---------------------------------------------------------------------------

TEST(PositionOutput, JacobianStructureC0) {
  auto xi = makeState();
  Eigen::Vector3d pi(2.0, -1.0, 1.5);
  auto C = PositionOutput::jacobian(xi, pi, PositionOutput::Variant::C0);

  const Eigen::Vector3d y_hat = PositionOutput::predict(xi, pi);
  EXPECT(meq(C.block<3, 3>(0, 0), skewSymmetric(y_hat)));          // d_theta
  EXPECT(meq(C.block<3, 3>(0, 3), Eigen::Matrix3d::Zero()));       // d_v
  EXPECT(meq(C.block<3, 3>(0, 6), -Eigen::Matrix3d::Identity()));  // d_p
  EXPECT(
      meq(C.block<3, 6>(0, 9), Eigen::Matrix<double, 3, 6>::Zero()));  // bias
}

// C* differs from C0 only in the d_theta block: 1/2 y_hat^ (this chart).
TEST(PositionOutput, JacobianStructureCstar) {
  auto xi = makeState();
  Eigen::Vector3d pi(2.0, -1.0, 1.5);
  auto C = PositionOutput::jacobian(xi, pi, PositionOutput::Variant::Cstar);

  const Eigen::Vector3d y_hat = PositionOutput::predict(xi, pi);
  EXPECT(meq(C.block<3, 3>(0, 0), skewSymmetric(0.5 * y_hat)));
  EXPECT(meq(C.block<3, 3>(0, 3), Eigen::Matrix3d::Zero()));
  EXPECT(meq(C.block<3, 3>(0, 6), -Eigen::Matrix3d::Identity()));
  EXPECT(meq(C.block<3, 6>(0, 9), Eigen::Matrix<double, 3, 6>::Zero()));
}

// When the residual vanishes (y_hat == 0, i.e. pi == p), C* reduces to C0
// (both d_theta blocks are zero).
TEST(PositionOutput, CstarReducesToC0WhenCentred) {
  TwoFrameGroup xi(Rot3::Rz(0.3), Eigen::Vector3d::Zero(),
                   Eigen::Vector3d(0.4, -0.2, 0.1), Eigen::Vector3d::Zero(),
                   Eigen::Vector3d::Zero());
  Eigen::Vector3d pi = xi.p;  // y_hat = R^T(pi - p) = 0
  auto C0 = PositionOutput::jacobian(xi, pi, PositionOutput::Variant::C0);
  auto Cs = PositionOutput::jacobian(xi, pi, PositionOutput::Variant::Cstar);
  EXPECT(meq(C0, Cs, kTolL));
}

// C0 must equal the numerical derivative of h(Retract(xi, eps)) at eps = 0,
// where Retract is the chart used by ManifoldEKF::update.
TEST(PositionOutput, JacobianMatchesFiniteDifference) {
  auto xi = makeState();
  Eigen::Vector3d pi(2.0, -1.0, 1.5);
  auto C = PositionOutput::jacobian(xi, pi, PositionOutput::Variant::C0);

  const Eigen::Matrix<double, 3, 15> C_num =
      numericalDerivative11<Eigen::Vector3d, TwoFrameGroup>(
          [&pi](const TwoFrameGroup& x) {
            return PositionOutput::predict(x, pi);
          },
          xi);
  EXPECT(meq(C, C_num, 1e-6));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
