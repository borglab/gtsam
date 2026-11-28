/**
 * @file  testTFGBodyVelocityOutput.cpp
 * @brief Unit tests for the unbiased DVL body-frame velocity output.
 */
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam_unstable/navigation/TFGInEKF.h>  // BodyVelocityOutput, phi

using namespace gtsam::tfg;
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
// predict:  h_d(xi) = R^T v
// ---------------------------------------------------------------------------

TEST(BodyVelocityOutput, PredictValue) {
  auto xi = makeState();
  EXPECT(veq(BodyVelocityOutput::predict(xi), xi.R.unrotate(xi.v)));
}

// ---------------------------------------------------------------------------
// outputAction + equivariance:  h_d(xi * X) == psi_d(X, h_d(xi))
// ---------------------------------------------------------------------------

TEST(BodyVelocityOutput, OutputEquivariance) {
  auto xi = makeState();
  auto X = TwoFrameGroup::FromState(
      Rot3::Ry(0.3) * Rot3::Rz(-0.1), Eigen::Vector3d(0.2, -0.3, 0.1),
      Eigen::Vector3d(-0.5, 1.0, 2.0), Eigen::Vector3d(0.0, 0.0, 0.0),
      Eigen::Vector3d(0.0, 0.0, 0.0));

  Eigen::Vector3d lhs = BodyVelocityOutput::predict(phi(X, xi));
  Eigen::Vector3d rhs =
      BodyVelocityOutput::outputAction(X, BodyVelocityOutput::predict(xi));
  EXPECT(veq(lhs, rhs, kTolL));
}

// ---------------------------------------------------------------------------
// jacobian: structure  H = [ y_hat^  I  0  0 ]  and FD correctness
// ---------------------------------------------------------------------------

TEST(BodyVelocityOutput, JacobianStructure) {
  auto xi = makeState();
  auto H = BodyVelocityOutput::jacobian(xi);

  const Eigen::Vector3d y_hat = BodyVelocityOutput::predict(xi);
  EXPECT(meq(H.block<3, 3>(0, 0), skewSymmetric(y_hat)));         // d_theta
  EXPECT(meq(H.block<3, 3>(0, 3), Eigen::Matrix3d::Identity()));  // d_v
  EXPECT(meq(H.block<3, 3>(0, 6), Eigen::Matrix3d::Zero()));      // d_p
  EXPECT(
      meq(H.block<3, 6>(0, 9), Eigen::Matrix<double, 3, 6>::Zero()));  // bias
}

// H must equal the numerical derivative of h_d(Retract(xi, eps)) at eps = 0,
// where Retract is the chart used by ManifoldEKF::update.
TEST(BodyVelocityOutput, JacobianMatchesFiniteDifference) {
  auto xi = makeState();
  auto H = BodyVelocityOutput::jacobian(xi);

  const Eigen::Matrix<double, 3, 15> H_num =
      numericalDerivative11<Eigen::Vector3d, TwoFrameGroup>(
          [](const TwoFrameGroup& x) { return BodyVelocityOutput::predict(x); },
          xi);
  EXPECT(meq(H, H_num, 1e-6));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
