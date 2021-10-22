/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testScenario.cpp
 * @brief   Unit test Scenario class
 * @author  Frank Dellaert
 */

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/navigation/Scenario.h>

#include <CppUnitLite/TestHarness.h>
#include <cmath>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

static const double kDegree = M_PI / 180.0;

/* ************************************************************************* */
TEST(Scenario, Spin) {
  //  angular velocity 6 kDegree/sec
  const double w = 6 * kDegree;
  const Vector3 W(0, 0, w), V(0, 0, 0);
  const ConstantTwistScenario scenario(W, V);

  const double T = 10;
  EXPECT(assert_equal(W, scenario.omega_b(T), 1e-9));
  EXPECT(assert_equal(V, scenario.velocity_b(T), 1e-9));
  EXPECT(assert_equal(W.cross(V), scenario.acceleration_b(T), 1e-9));

  const Pose3 T10 = scenario.pose(T);
  EXPECT(assert_equal(Vector3(0, 0, 60 * kDegree), T10.rotation().xyz(), 1e-9));
  EXPECT(assert_equal(Point3(0, 0, 0), T10.translation(), 1e-9));
}

/* ************************************************************************* */
TEST(Scenario, Forward) {
  const double v = 2;  // m/s
  const Vector3 W(0, 0, 0), V(v, 0, 0);
  const ConstantTwistScenario scenario(W, V);

  const double T = 15;
  EXPECT(assert_equal(W, scenario.omega_b(T), 1e-9));
  EXPECT(assert_equal(V, scenario.velocity_b(T), 1e-9));
  EXPECT(assert_equal(W.cross(V), scenario.acceleration_b(T), 1e-9));

  const Pose3 T15 = scenario.pose(T);
  EXPECT(assert_equal(Vector3(0, 0, 0), T15.rotation().xyz(), 1e-9));
  EXPECT(assert_equal(Point3(30, 0, 0), T15.translation(), 1e-9));
}

/* ************************************************************************* */
TEST(Scenario, Circle) {
  // Forward velocity 2m/s, angular velocity 6 kDegree/sec around Z
  const double v = 2, w = 6 * kDegree;
  const Vector3 W(0, 0, w), V(v, 0, 0);
  const ConstantTwistScenario scenario(W, V);

  const double T = 15;
  EXPECT(assert_equal(W, scenario.omega_b(T), 1e-9));
  EXPECT(assert_equal(V, scenario.velocity_b(T), 1e-9));
  EXPECT(assert_equal(W.cross(V), scenario.acceleration_b(T), 1e-9));

  // R = v/w, so test if circle is of right size
  const double R = v / w;
  const Pose3 T15 = scenario.pose(T);
  EXPECT(assert_equal(Vector3(0, 0, 90 * kDegree), T15.rotation().xyz(), 1e-9));
  EXPECT(assert_equal(Point3(R, R, 0), T15.translation(), 1e-9));
}

/* ************************************************************************* */
TEST(Scenario, Loop) {
  // Forward velocity 2m/s
  // Pitch up with angular velocity 6 kDegree/sec (negative in FLU)
  const double v = 2, w = 6 * kDegree;
  const Vector3 W(0, -w, 0), V(v, 0, 0);
  const ConstantTwistScenario scenario(W, V);

  const double T = 30;
  EXPECT(assert_equal(W, scenario.omega_b(T), 1e-9));
  EXPECT(assert_equal(V, scenario.velocity_b(T), 1e-9));
  EXPECT(assert_equal(W.cross(V), scenario.acceleration_b(T), 1e-9));

  // R = v/w, so test if loop crests at 2*R
  const double R = v / w;
  const Pose3 T30 = scenario.pose(30);
  EXPECT(assert_equal(Rot3::Rodrigues(0, M_PI, 0), T30.rotation(), 1e-9));
#ifdef GTSAM_USE_QUATERNIONS
  EXPECT(assert_equal(Vector3(-M_PI, 0, -M_PI), T30.rotation().xyz()));
#else
  EXPECT(assert_equal(Vector3(M_PI, 0, M_PI), T30.rotation().xyz()));
#endif
  EXPECT(assert_equal(Point3(0, 0, 2 * R), T30.translation(), 1e-9));
}

/* ************************************************************************* */
TEST(Scenario, LoopWithInitialPose) {
  // Forward velocity 2m/s
  // Pitch up with angular velocity 6 kDegree/sec (negative in FLU)
  const double v = 2, w = 6 * kDegree;
  const Vector3 W(0, -w, 0), V(v, 0, 0);
  const Rot3 nRb0 = Rot3::Yaw(M_PI);
  const Pose3 nTb0(nRb0, Point3(1, 2, 3));
  const ConstantTwistScenario scenario(W, V, nTb0);

  const double T = 30;
  EXPECT(assert_equal(W, scenario.omega_b(T), 1e-9));
  EXPECT(assert_equal(V, scenario.velocity_b(T), 1e-9));
  EXPECT(assert_equal(W.cross(V), scenario.acceleration_b(T), 1e-9));

  // R = v/w, so test if loop crests at 2*R
  const double R = v / w;
  const Pose3 T30 = scenario.pose(30);
  EXPECT(
      assert_equal(nRb0 * Rot3::Rodrigues(0, M_PI, 0), T30.rotation(), 1e-9));
  EXPECT(assert_equal(Point3(1, 2, 3 + 2 * R), T30.translation(), 1e-9));
}

/* ************************************************************************* */
TEST(Scenario, Accelerating) {
  // Set up body pointing towards y axis, and start at 10,20,0 with velocity
  // going in X. The body itself has Z axis pointing down
  const Rot3 nRb(Point3(0, 1, 0), Point3(1, 0, 0), Point3(0, 0, -1));
  const Point3 P0(10, 20, 0);
  const Vector3 V0(50, 0, 0);

  const double a = 0.2;  // m/s^2
  const Vector3 A(0, a, 0), W(0.1, 0.2, 0.3);
  const AcceleratingScenario scenario(nRb, P0, V0, A, W);

  const double T = 3;
  EXPECT(assert_equal(W, scenario.omega_b(T), 1e-9));
  EXPECT(assert_equal(Vector3(V0 + T * A), scenario.velocity_n(T), 1e-9));
  EXPECT(assert_equal(A, scenario.acceleration_n(T), 1e-9));

  {
    // Check acceleration in nav
    Matrix expected = numericalDerivative11<Vector3, double>(
        std::bind(&Scenario::velocity_n, scenario, std::placeholders::_1), T);
    EXPECT(assert_equal(Vector3(expected), scenario.acceleration_n(T), 1e-9));
  }

  const Pose3 T3 = scenario.pose(3);
  EXPECT(assert_equal(nRb.expmap(T * W), T3.rotation(), 1e-9));
  EXPECT(assert_equal(Point3(10 + T * 50, 20 + a * T * T / 2, 0),
                      T3.translation(), 1e-9));
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
