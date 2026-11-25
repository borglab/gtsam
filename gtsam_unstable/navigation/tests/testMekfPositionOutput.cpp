/**
 * @file  testMekfPositionOutput.cpp
 * @brief Unit tests for the naive world-frame position output (h=p, H, FD
 * check).
 *
 * Tangent block order: R, v, p, b_gyro, b_accel.
 */
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam_unstable/navigation/MEKF.h>

using namespace mekf;
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

static MekfState exampleState() {
  return MekfState(
      Rot3::Rz(0.4) * Rot3::Rx(0.2), Eigen::Vector3d(1.0, -0.5, 0.3),
      Eigen::Vector3d(2.0, 1.0, -1.0), Eigen::Vector3d(0.01, -0.02, 0.03),
      Eigen::Vector3d(-0.05, 0.1, 0.0));
}

// ---------------------------------------------------------------------------
// predict
// ---------------------------------------------------------------------------

// h(xi) = p (the global position), independent of attitude / velocity / bias.
TEST(MekfPositionOutput, PredictReturnsPosition) {
  const MekfState X = exampleState();
  EXPECT(veq(PositionOutput::predict(X), X.p));
}

// ---------------------------------------------------------------------------
// jacobian
// ---------------------------------------------------------------------------

// H selects the d_p block: H = [0 0 I 0 0].
TEST(MekfPositionOutput, JacobianStructure) {
  Eigen::Matrix<double, 3, 15> H = PositionOutput::jacobian();
  Eigen::Matrix<double, 3, 15> expected = Eigen::Matrix<double, 3, 15>::Zero();
  expected.block<3, 3>(0, 6) = Eigen::Matrix3d::Identity();
  EXPECT(meq(H, expected));
}

TEST(MekfPositionOutput, JacobianIsConstant) {
  EXPECT(meq(PositionOutput::jacobian(), PositionOutput::jacobian()));
}

// Finite-difference check: H = d h(Retract(X, eps)) / d eps at eps = 0.
TEST(MekfPositionOutput, JacobianMatchesFiniteDifference) {
  const MekfState X = exampleState();
  const Eigen::Matrix<double, 3, 15> H = PositionOutput::jacobian();

  const Eigen::Matrix<double, 3, 15> Hfd =
      numericalDerivative11<Eigen::Vector3d, MekfState>(
          [](const MekfState& x) { return PositionOutput::predict(x); }, X);
  EXPECT(meq(H, Hfd, 1e-6));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
