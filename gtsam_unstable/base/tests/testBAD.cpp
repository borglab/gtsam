/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testBAD.cpp
 * @date September 18, 2014
 * @author Frank Dellaert
 * @author Paul Furgale
 * @brief unit tests for Block Automatic Differentiation
 */

#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam_unstable/base/Expression.h>
#include <gtsam/base/Testable.h>

#include <CppUnitLite/TestHarness.h>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */

Point3 transformTo(const Pose3& x, const Point3& p,
    boost::optional<Matrix&> Dpose, boost::optional<Matrix&> Dpoint) {
  return x.transform_to(p, Dpose, Dpoint);
}

Point2 project(const Point3& p, boost::optional<Matrix&> Dpoint) {
  return PinholeCamera<Cal3_S2>::project_to_camera(p, Dpoint);
}

template<class CAL>
Point2 uncalibrate(const CAL& K, const Point2& p, boost::optional<Matrix&> Dcal,
    boost::optional<Matrix&> Dp) {
  return K.uncalibrate(p, Dcal, Dp);
}

/* ************************************************************************* */

TEST(BAD, test) {

  // Test Constant expression
  Expression<int> c(0);

  // Create leaves
  Expression<Pose3> x(1);
  Expression<Point3> p(2);
  Expression<Cal3_S2> K(3);

  // Create expression tree
  Expression<Point3> p_cam(transformTo, x, p);
  Expression<Point2> projection(project, p_cam);
  Expression<Point2> uv_hat(uncalibrate<Cal3_S2>, K, projection);

  // Check keys
  std::set<Key> expectedKeys;
  expectedKeys.insert(1);
  expectedKeys.insert(2);
  expectedKeys.insert(3);
  EXPECT(expectedKeys == uv_hat.keys());
}

/* ************************************************************************* */

TEST(BAD, compose) {

  // Create expression
  Expression<Rot3> R1(1), R2(2);
  Expression<Rot3> R3 = R1 * R2;

  // Check keys
  std::set<Key> expectedKeys;
  expectedKeys.insert(1);
  expectedKeys.insert(2);
  EXPECT(expectedKeys == R3.keys());
}

/* ************************************************************************* */
// Test compose with arguments referring to the same rotation
TEST(BAD, compose2) {

  // Create expression
  Expression<Rot3> R1(1), R2(1);
  Expression<Rot3> R3 = R1 * R2;

  // Check keys
  std::set<Key> expectedKeys;
  expectedKeys.insert(1);
  EXPECT(expectedKeys == R3.keys());
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */

