/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    Scenario.h
 * @brief   Simple class to test navigation scenarios
 * @author  Frank Dellaert
 */

#include <gtsam/geometry/Pose3.h>

namespace gtsam {
/// Simple class to test navigation scenarios
class Scenario {
 public:
  /// Construct scenario with constant twist [w,v]
  Scenario(const Vector3& w, const Vector3& v)
      : twist_((Vector6() << w, v).finished()) {}

  Pose3 poseAtTime(double t) { return Pose3::Expmap(twist_ * t); }

 private:
  Vector6 twist_;
};

}  // namespace gtsam

/**
 * @file    testScenario.cpp
 * @brief   test ImuFacor with Scenario class
 * @author  Frank Dellaert
 */

#include <CppUnitLite/TestHarness.h>

#include <cmath>

using namespace std;
using namespace gtsam;

static const double degree = M_PI / 180.0;

/* ************************************************************************* */
TEST(Scenario, Circle) {
  // Forward velocity 2m/s, angular velocity 6 degree/sec
  const double v = 2, omega = 6 * degree;
  Scenario circle(Vector3(0, 0, omega), Vector3(v, 0, 0));

  // R = v/omega, so test if circle is of right size
  const double R = v / omega;
  const Pose3 T15 = circle.poseAtTime(15);
  EXPECT(assert_equal(Vector3(0, 0, 90 * degree), T15.rotation().xyz(), 1e-9));
  EXPECT(assert_equal(Point3(R, R, 0), T15.translation(), 1e-9));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
