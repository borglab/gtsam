/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file    testPoseVelocityBias.cpp
 * @brief   Unit test for PoseVelocityBias
 * @author  Frank Dellaert
 */

#include <gtsam/navigation/PreintegrationBase.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/factorTesting.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/numericalDerivative.h>
#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

#ifdef GTSAM_ALLOW_DEPRECATED_SINCE_V4

// Should be seen as between(pvb1,pvb2), i.e., written as pvb2 \omin pvb1
Vector9 error(const PoseVelocityBias& pvb1, const PoseVelocityBias& pvb2) {
  Matrix3 R1 = pvb1.pose.rotation().matrix();
  // Ri.transpose() translate the error from the global frame into pose1's frame
  const Vector3 fp = R1.transpose() * (pvb2.pose.translation() - pvb1.pose.translation());
  const Vector3 fv = R1.transpose() * (pvb2.velocity - pvb1.velocity);
  const Rot3 R1BetweenR2 = pvb1.pose.rotation().between(pvb2.pose.rotation());
  const Vector3 fR = Rot3::Logmap(R1BetweenR2);
  Vector9 r;
  r << fp, fv, fR;
  return r;
}

/* ************************************************************************************************/
TEST(PoseVelocityBias, error) {
  Point3 i1(0, 1, 0), j1(-1, 0, 0), k1(0, 0, 1);
  Pose3 x1(Rot3(i1, j1, k1), Point3(5.0, 1.0, 0.0));
  Vector3 v1(Vector3(0.5, 0.0, 0.0));
  imuBias::ConstantBias bias1(Vector3(0.2, 0, 0), Vector3(0.1, 0, 0.3));

  Pose3 x2(Rot3(i1, j1, k1).expmap(Vector3(0.1, 0.2, 0.3)), Point3(5.5, 1.0, 6.0));
  Vector3 v2(Vector3(0.5, 4.0, 3.0));
  imuBias::ConstantBias bias2(Vector3(0.1, 0.2, -0.3), Vector3(0.2, 0.3, 0.1));

  PoseVelocityBias pvb1(x1, v1, bias1), pvb2(x2, v2, bias2);

  Vector9 expected, actual = error(pvb1, pvb2);
  expected << 0.0, -0.5, 6.0, 4.0, 0.0, 3.0, 0.1, 0.2, 0.3;
  EXPECT(assert_equal(expected, actual, 1e-9));
}
#endif

/* ************************************************************************************************/
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************************************/
