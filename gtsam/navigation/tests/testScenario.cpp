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
#include <fstream>
#include <cstdio>

using namespace std::placeholders;
using namespace std;
using namespace gtsam;

static const double kDegree = M_PI / 180.0;

// Helper function to create a common DiscreteScenario for multiple tests.
// The scenario has ground truth at t=0 and t=2.
static DiscreteScenario CreateSimpleDiscreteScenario() {
  const double t0 = 0.0, t1 = 2.0;

  // Data at t=0
  const Pose3 pose0;  // Identity
  const Vector3 omega0 = Vector3::Zero();
  const Vector3 vel0(1, 0, 0);
  const Vector3 acc0(0, 1, 0);

  // Data at t=2
  const Rot3 R1 = Rot3::Rodrigues(0, 0, 0.2); // Rotates by 0.2 rad around Z
  const Pose3 pose1(R1, Point3(2, 2, 0));
  const Vector3 omega1(0, 0, 0.1);
  const Vector3 vel1(1, 2, 0);
  const Vector3 acc1(0, 1, 0); // Constant acceleration in nav frame

  map<double, Pose3> poses;
  poses[t0] = pose0;
  poses[t1] = pose1;

  map<double, Vector3> omegas;
  omegas[t0] = omega0;
  omegas[t1] = omega1;

  map<double, Vector3> velocities;
  velocities[t0] = vel0;
  velocities[t1] = vel1;

  map<double, Vector3> accelerations;
  accelerations[t0] = acc0;
  accelerations[t1] = acc1;

  return DiscreteScenario(poses, omegas, velocities, accelerations);
}

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
TEST(DiscreteScenario, EndpointsAndClamping) {
  // This test verifies that the scenario correctly returns values at the exact
  // timestamps provided, and clamps to those endpoint values for times
  // outside the defined range.
  auto scenario = CreateSimpleDiscreteScenario();

  // Expected endpoint values
  const Pose3 pose0, pose1(Rot3::Rodrigues(0, 0, 0.2), Point3(2, 2, 0));
  const Vector3 omega0 = Vector3::Zero(), omega1(0, 0, 0.1);
  const Vector3 vel0(1, 0, 0), vel1(1, 2, 0);
  const Vector3 acc0(0, 1, 0), acc1(0, 1, 0);

  // --- Test at start endpoint (t=0) ---
  EXPECT(assert_equal(pose0, scenario.pose(0.0)));
  EXPECT(assert_equal(omega0, scenario.omega_b(0.0)));
  EXPECT(assert_equal(vel0, scenario.velocity_n(0.0)));
  EXPECT(assert_equal(acc0, scenario.acceleration_n(0.0)));

  // --- Test at end endpoint (t=2) ---
  EXPECT(assert_equal(pose1, scenario.pose(2.0)));
  EXPECT(assert_equal(omega1, scenario.omega_b(2.0)));
  EXPECT(assert_equal(vel1, scenario.velocity_n(2.0)));
  EXPECT(assert_equal(acc1, scenario.acceleration_n(2.0)));

  // --- Test clamping (before start) ---
  EXPECT(assert_equal(pose0, scenario.pose(-1.0)));
  EXPECT(assert_equal(omega0, scenario.omega_b(-1.0)));
  EXPECT(assert_equal(vel0, scenario.velocity_n(-1.0)));
  EXPECT(assert_equal(acc0, scenario.acceleration_n(-1.0)));

  // --- Test clamping (after end) ---
  EXPECT(assert_equal(pose1, scenario.pose(3.0)));
  EXPECT(assert_equal(omega1, scenario.omega_b(3.0)));
  EXPECT(assert_equal(vel1, scenario.velocity_n(3.0)));
  EXPECT(assert_equal(acc1, scenario.acceleration_n(3.0)));
}

/* ************************************************************************* */
TEST(DiscreteScenario, Interpolation) {
  // This test verifies the interpolation logic for a time between endpoints.
  auto scenario = CreateSimpleDiscreteScenario();

  // Expected endpoint values to interpolate between
  const Pose3 pose0, pose1(Rot3::Rodrigues(0, 0, 0.2), Point3(2, 2, 0));
  const Vector3 omega0 = Vector3::Zero(), omega1(0, 0, 0.1);
  const Vector3 vel0(1, 0, 0), vel1(1, 2, 0);
  const Vector3 acc0(0, 1, 0), acc1(0, 1, 0);

  // --- Test interpolation at midpoint (t=1.0) ---
  const double t_mid = 1.0;
  // Alpha for interpolation is (1.0 - 0.0) / (2.0 - 0.0) = 0.5
  const double alpha = 0.5;
  const Pose3 expected_pose_mid = interpolate(pose0, pose1, alpha);
  const Vector3 expected_omega_mid = interpolate(omega0, omega1, alpha);
  const Vector3 expected_vel_mid = interpolate(vel0, vel1, alpha);
  const Vector3 expected_acc_mid = interpolate(acc0, acc1, alpha);

  EXPECT(assert_equal(expected_pose_mid, scenario.pose(t_mid), 1e-6));
  EXPECT(assert_equal(expected_omega_mid, scenario.omega_b(t_mid), 1e-9));
  EXPECT(assert_equal(expected_vel_mid, scenario.velocity_n(t_mid), 1e-9));
  EXPECT(assert_equal(expected_acc_mid, scenario.acceleration_n(t_mid), 1e-9));
}

/* ************************************************************************* */
TEST(DiscreteScenario, ConstructorExceptions) {
  // This test verifies that the constructor correctly throws an exception when
  // provided with any empty data map.
  map<double, Pose3> poses;
  poses[0.0] = Pose3();
  map<double, Vector3> omegas;
  omegas[0.0] = Vector3::Zero();
  map<double, Vector3> velocities;
  velocities[0.0] = Vector3::Zero();
  map<double, Vector3> accelerations;
  accelerations[0.0] = Vector3::Zero();

  map<double, Pose3> empty_poses;
  map<double, Vector3> empty_vectors;

  CHECK_EXCEPTION(DiscreteScenario(empty_poses, omegas, velocities, accelerations),
                   std::invalid_argument);
  CHECK_EXCEPTION(DiscreteScenario(poses, empty_vectors, velocities, accelerations),
                   std::invalid_argument);
  CHECK_EXCEPTION(DiscreteScenario(poses, omegas, empty_vectors, accelerations),
                   std::invalid_argument);
  CHECK_EXCEPTION(DiscreteScenario(poses, omegas, velocities, empty_vectors),
                   std::invalid_argument);
}

/* ************************************************************************* */
TEST(DiscreteScenario, FromCSV_Correct) {
  // This test checks the successful creation of a scenario from a valid CSV
  // file, including timestamp normalization and correct interpolation.
  const string csv_filepath = "tmp_scenario_test.csv";
  ofstream csv_file(csv_filepath);
  CHECK(csv_file.is_open());

  // Header and data
  csv_file << "t,p_x,p_y,p_z,q_w,q_x,q_y,q_z,v_x,v_y,v_z,w_x,w_y,w_z,a_x,a_y,a_z\n";
  csv_file << "100.0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,1,0\n"; // t_norm = 0.0
  const Rot3 R1 = Rot3::Rodrigues(0, 0, 0.2);
  const Quaternion q1 = R1.toQuaternion();
  csv_file << "102.0,2,2,0," << q1.w() << "," << q1.x() << "," << q1.y() << "," << q1.z()
           << ",1,2,0,0,0,0.1,0,1,0\n"; // t_norm = 2.0
  csv_file.close();

  // Load the scenario and check values, which should be the same as map-based tests
  auto scenario = DiscreteScenario::FromCSV(csv_filepath);
  
  // Expected endpoint values after normalization
  const Pose3 pose0, pose1(R1, Point3(2, 2, 0));
  const Vector3 vel0(1, 0, 0), vel1(1, 2, 0);

  // Test at endpoints (using normalized timestamps)
  EXPECT(assert_equal(pose0, scenario.pose(0.0), 1e-6));
  EXPECT(assert_equal(vel0, scenario.velocity_n(0.0)));
  EXPECT(assert_equal(pose1, scenario.pose(2.0), 1e-6));
  EXPECT(assert_equal(vel1, scenario.velocity_n(2.0)));
  
  // Test interpolation
  const Pose3 expected_pose_mid = interpolate(pose0, pose1, 0.5);
  EXPECT(assert_equal(expected_pose_mid, scenario.pose(1.0), 1e-6));

  // Clean up
  remove(csv_filepath.c_str());
}

/* ************************************************************************* */
TEST(DiscreteScenario, FromCSV_Exceptions) {
  // This test ensures that FromCSV throws runtime_errors for invalid file
  // paths or malformed file content.
  
  // --- Test non-existent file ---
  CHECK_EXCEPTION(DiscreteScenario::FromCSV("non_existent_file.csv"),
                   std::runtime_error);

  // --- Test malformed file (not enough columns) ---
  const string malformed_filepath = "tmp_malformed_test.csv";
  ofstream malformed_file(malformed_filepath);
  CHECK(malformed_file.is_open());
  malformed_file << "timestamp,px,py,pz\n";
  malformed_file << "100.0,0,0,0\n";
  malformed_file.close();

  CHECK_EXCEPTION(DiscreteScenario::FromCSV(malformed_filepath),
                   std::runtime_error);

  // Clean up
  remove(malformed_filepath.c_str());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
