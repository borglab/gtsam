/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <CppUnitLite/TestHarness.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/TestableAssertions.h>
#include <gtsam_unstable/slam/MagPoseFactor.h>

using namespace gtsam;

// Magnetic field in the nav frame (NED), with units of nT.
Point3 nM(22653.29982, -1956.83010, 44202.47862);

// Assumed scale factor (scales a unit vector to magnetometer units of nT).
double scale = 255.0 / 50000.0;

// Ground truth Pose2/Pose3 in the nav frame.
Pose3 n_P3_b = Pose3(Rot3::Yaw(-0.1), Point3(-3, 12, 5));
Pose2 n_P2_b = Pose2(Rot2(-0.1), Point2(-3, 12));
Rot3 nRb = n_P3_b.rotation();
Rot2 theta = n_P2_b.rotation();

// Sensor bias (nT).
Point3 bias3(10, -10, 50);
Point2 bias2 = bias3.head(2);

// double s(scale * nM.norm());
Point2 dir2(nM.head(2).normalized());
Point3 dir3(nM.normalized());

// Compute the measured field in the sensor frame.
Point3 measured3 = nRb.inverse() * (scale * dir3) + bias3;

// The 2D magnetometer will measure the "NE" field components.
Point2 measured2 = theta.inverse() * (scale * dir2) + bias2;

SharedNoiseModel model2 = noiseModel::Isotropic::Sigma(2, 0.25);
SharedNoiseModel model3 = noiseModel::Isotropic::Sigma(3, 0.25);

// Make up a rotation and offset of the sensor in the body frame.
Pose2 body_P2_sensor(Rot2(-0.30), Point2(1, -2));
Pose3 body_P3_sensor(Rot3::RzRyRx(Vector3(0.4, 0.1, -0.15)), Point3(-0.1, 0.2, 0.3));

// *****************************************************************************
TEST(MagPoseFactor, Constructors) {
  MagPoseFactor<Pose2> f2a(Symbol('X', 0), measured2, scale, dir2, bias2, model2, boost::none);
  MagPoseFactor<Pose3> f3a(Symbol('X', 0), measured3, scale, dir3, bias3, model3, boost::none);

  // Try constructing with a body_P_sensor set.
  MagPoseFactor<Pose2> f2b = MagPoseFactor<Pose2>(Symbol('X', 0), measured2, scale, dir2, bias2, model2, body_P2_sensor);
  MagPoseFactor<Pose3> f3b = MagPoseFactor<Pose3>(Symbol('X', 0), measured3, scale, dir3, bias3, model3, body_P3_sensor);
}

// *****************************************************************************
TEST(MagPoseFactor, JacobianPose2) {
  Matrix H2;

  // Error should be zero at the groundtruth pose.
  MagPoseFactor<Pose2> f2a(Symbol('X', 0), measured2, scale, dir2, bias2, model2, boost::none);
  CHECK(gtsam::assert_equal(Z_2x1, f2a.evaluateError(n_P2_b, H2), 1e-5));
  CHECK(gtsam::assert_equal(gtsam::numericalDerivative11<Vector, Pose2> //
      (boost::bind(&MagPoseFactor<Pose2>::evaluateError, &f2a, _1, boost::none), n_P2_b), H2, 1e-7));

  // Now test with a body_P_sensor specified, which means the raw measurement
  // must be rotated into the sensor frame.
  MagPoseFactor<Pose2> f2b = MagPoseFactor<Pose2>(Symbol('X', 0),
      body_P2_sensor.rotation().inverse() * measured2, scale, dir2, bias2, model2, body_P2_sensor);
  CHECK(gtsam::assert_equal(Z_2x1, f2b.evaluateError(n_P2_b, H2), 1e-5));
  CHECK(gtsam::assert_equal(gtsam::numericalDerivative11<Vector, Pose2> //
      (boost::bind(&MagPoseFactor<Pose2>::evaluateError, &f2b, _1, boost::none), n_P2_b), H2, 1e-7));
}

// *****************************************************************************
TEST(MagPoseFactor, JacobianPose3) {
  Matrix H3;

  // Error should be zero at the groundtruth pose.
  MagPoseFactor<Pose3> f3a(Symbol('X', 0), measured3, scale, dir3, bias3, model3, boost::none);
  CHECK(gtsam::assert_equal(Z_3x1, f3a.evaluateError(n_P3_b, H3), 1e-5));
  CHECK(gtsam::assert_equal(gtsam::numericalDerivative11<Vector, Pose3> //
      (boost::bind(&MagPoseFactor<Pose3>::evaluateError, &f3a, _1, boost::none), n_P3_b), H3, 1e-7));

  // Now test with a body_P_sensor specified, which means the raw measurement
  // must be rotated into the sensor frame.
  MagPoseFactor<Pose3> f3b = MagPoseFactor<Pose3>(Symbol('X', 0),
      body_P3_sensor.rotation().inverse() * measured3, scale, dir3, bias3, model3, boost::none);
  CHECK(gtsam::assert_equal(Z_3x1, f3b.evaluateError(n_P3_b, H3), 1e-5));
  CHECK(gtsam::assert_equal(gtsam::numericalDerivative11<Vector, Pose3> //
      (boost::bind(&MagPoseFactor<Pose3>::evaluateError, &f3b, _1, boost::none), n_P3_b), H3, 1e-7));
}

// *****************************************************************************
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
// *****************************************************************************
