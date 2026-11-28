/**
 * @file  testMEKFBodyVelocityOutput.cpp
 * @brief Unit tests for the DVL body-frame velocity output (h=R^T v, H, FD).
 *
 * Tangent block order: R, v, p, b_gyro, b_accel.
 */
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam_unstable/navigation/MEKF.h>

using namespace gtsam::mekf;
using namespace gtsam;

static constexpr double kTol = 1e-9;

static bool veq(const Eigen::Vector3d& a, const Eigen::Vector3d& b,
                double tol = kTol) {
  return assert_equal((Vector)a, (Vector)b, tol);
}
static bool meq(const Eigen::MatrixXd& a, const Eigen::MatrixXd& b,
                double tol = kTol) {
  return assert_equal((Matrix)a, (Matrix)b, tol);
}

static MEKFState exampleState() {
  return MEKFState(
      Rot3::Rz(0.4) * Rot3::Rx(0.2), Eigen::Vector3d(1.0, -0.5, 0.3),
      Eigen::Vector3d(2.0, 1.0, -1.0), Eigen::Vector3d(0.01, -0.02, 0.03),
      Eigen::Vector3d(-0.05, 0.1, 0.0));
}

// ---------------------------------------------------------------------------
// predict
// ---------------------------------------------------------------------------

// h(xi) = R^T v (world velocity rotated into the body frame).
TEST(MEKFBodyVelocityOutput, PredictReturnsBodyVelocity) {
  const MEKFState X = exampleState();
  EXPECT(veq(BodyVelocityOutput::predict(X), X.R.unrotate(X.v)));
}

// ---------------------------------------------------------------------------
// jacobian
// ---------------------------------------------------------------------------

// H = [ [R^T v]^x | R^T | 0 | 0 | 0 ]; other blocks vanish.
TEST(MEKFBodyVelocityOutput, JacobianStructure) {
  const MEKFState X = exampleState();
  const Eigen::Matrix<double, 3, 15> H = BodyVelocityOutput::jacobian(X);

  Eigen::Matrix<double, 3, 15> expected = Eigen::Matrix<double, 3, 15>::Zero();
  expected.block<3, 3>(0, 0) = skewSymmetric(X.R.unrotate(X.v));
  expected.block<3, 3>(0, 3) = X.R.matrix().transpose();
  EXPECT(meq(H, expected));
}

// Finite-difference check: H = d h(Retract(X, eps)) / d eps at eps = 0.
TEST(MEKFBodyVelocityOutput, JacobianMatchesFiniteDifference) {
  const MEKFState X = exampleState();
  const Eigen::Matrix<double, 3, 15> H = BodyVelocityOutput::jacobian(X);

  const Eigen::Matrix<double, 3, 15> Hfd =
      numericalDerivative11<Eigen::Vector3d, MEKFState>(
          [](const MEKFState& x) { return BodyVelocityOutput::predict(x); }, X);
  EXPECT(meq(H, Hfd, 1e-6));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
