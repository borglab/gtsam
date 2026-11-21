/**
 * @file  testSplineScenarioAttitude.cpp
 * @brief Attitude profile added to SplineScenario (IMUScenarios.h): the
 *        synthesized gyro must stay kinematically consistent with the
 *        interpolated rotation, and adding attitude must not disturb
 *        position/velocity/acceleration or the no-attitude behaviour.
 */
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam_unstable/examples_common/IMUScenarios.h>

using namespace gtsam;
using namespace imu_scenarios;

/* ************************************************************************* */
namespace fixture {

constexpr double kPathDuration = 40.0;
constexpr double kRamp = kRampDuration;

std::vector<Vector3> waypoints() {
  return {
      Vector3(0.0, 0.0, 0.0),   Vector3(8.0, 2.0, 1.0),
      Vector3(16.0, -1.0, 2.0), Vector3(24.0, 3.0, 0.5),
      Vector3(32.0, 0.0, -1.0), Vector3(40.0, 1.0, 0.0),
  };
}

// Roll oscillates, pitch ramps gently, yaw ramps continuously and past pi --
// exercising the unwrapped-yaw path with no re-wrapping.
std::vector<Vector3> attitudes() {
  std::vector<Vector3> out;
  const auto wp = waypoints();
  for (std::size_t i = 0; i < wp.size(); ++i) {
    const double x = static_cast<double>(i);
    out.push_back(Vector3(0.2 * std::sin(0.9 * x), 0.05 * x, 0.7 * x));
  }
  return out;
}

Rot3 eulerToRot3(const Vector3& rpy) {
  return Rot3::Ypr(rpy(2), rpy(1), rpy(0));
}

}  // namespace fixture

/* ************************************************************************* */
TEST(SplineScenarioAttitude, RdotMatchesOmegaCrossProduct) {
  const SplineScenario scenario(fixture::waypoints(), fixture::attitudes(),
                                fixture::kPathDuration, fixture::kRamp);
  const double h = 1e-4;
  // Central-difference error is O(h^2); with h=1e-4 and the smooth cubic
  // splines here the residual is well under 1e-6, so this stays tight.
  const double tol = 1e-6;
  for (double t : {3.0, 8.0, 15.0, 25.0}) {
    const Matrix3 Rp = scenario.pose(t + h).rotation().matrix();
    const Matrix3 Rm = scenario.pose(t - h).rotation().matrix();
    const Matrix3 RdotFd = (Rp - Rm) / (2.0 * h);
    const Matrix3 R = scenario.pose(t).rotation().matrix();
    const Vector3 omega = scenario.omega_b(t);
    const Matrix3 RdotAnalytic = R * skewSymmetric(omega);
    EXPECT(assert_equal(RdotFd, RdotAnalytic, tol));
  }
}

/* ************************************************************************* */
TEST(SplineScenarioAttitude, InitialOmegaAndAttitudeAreZeroed) {
  const SplineScenario scenario(fixture::waypoints(), fixture::attitudes(),
                                fixture::kPathDuration, fixture::kRamp);
  EXPECT(assert_equal<Vector3>(Vector3::Zero(), scenario.omega_b(0.0), 1e-12));
  const Rot3 expected = fixture::eulerToRot3(fixture::attitudes().front());
  EXPECT(assert_equal(expected, scenario.pose(0.0).rotation(), 1e-9));
}

/* ************************************************************************* */
TEST(SplineScenarioAttitude, BackwardCompatibleWithNoAttitude) {
  const SplineScenario scenario(fixture::waypoints(), {},
                                fixture::kPathDuration, fixture::kRamp);
  for (double t : {0.0, 3.0, 8.0, 20.0, 50.0}) {
    EXPECT(assert_equal(Rot3(), scenario.pose(t).rotation(), 1e-12));
    EXPECT(assert_equal<Vector3>(Vector3::Zero(), scenario.omega_b(t), 1e-12));
  }
}

/* ************************************************************************* */
TEST(SplineScenarioAttitude, PositionVelocityAccelerationUnaffectedByAttitude) {
  const SplineScenario withAttitude(fixture::waypoints(), fixture::attitudes(),
                                    fixture::kPathDuration, fixture::kRamp);
  const SplineScenario without(fixture::waypoints(), {}, fixture::kPathDuration,
                               fixture::kRamp);
  for (double t : {0.0, 1.0, 3.0, 8.0, 15.0, 25.0, 45.0}) {
    EXPECT(assert_equal(without.pose(t).translation(),
                        withAttitude.pose(t).translation(), 1e-12));
    EXPECT(
        assert_equal(without.velocity_n(t), withAttitude.velocity_n(t), 1e-12));
    EXPECT(assert_equal(without.acceleration_n(t),
                        withAttitude.acceleration_n(t), 1e-12));
  }
}

/* ************************************************************************* */
TEST(SplineScenarioAttitude, UniformKnotsAdvanceByIndexNotArcLength) {
  // Waypoints bunched near the start: chord-length u for index 2 sits near 0,
  // far from 0.5. Uniform knots put every waypoint at u_i = i / (n - 1)
  // regardless of spacing, so index 2 (of 5) lands exactly at u = 0.5.
  const std::vector<Vector3> wp = {
      Vector3(0.0, 0.0, 0.0),   Vector3(0.01, 0.0, 0.0),
      Vector3(0.02, 0.0, 0.0),  Vector3(50.0, 0.0, 0.0),
      Vector3(100.0, 0.0, 0.0),
  };
  const double pathDuration = 8.0;
  const double rampDuration = 0.0;  // S(t) = t exactly, so u = t / pathDuration
  const double tHalf = 0.5 * pathDuration;

  const SplineScenario uniform(wp, {}, pathDuration, rampDuration,
                               SplineScenario::KnotParameterization::Uniform);
  EXPECT(assert_equal(wp[2], uniform.pose(tHalf).translation(), 1e-9));

  const SplineScenario chordLength(
      wp, {}, pathDuration, rampDuration,
      SplineScenario::KnotParameterization::ChordLength);
  const Vector3 chordAtHalf = chordLength.pose(tHalf).translation();
  EXPECT((wp[2] - chordAtHalf).norm() > 1.0);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
