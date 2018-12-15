/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation,
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file testSlamExpressions.cpp
 * @author Frank Dellaert
 * @brief test expressions for SLAM
 */

#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/expressionTesting.h>
#include <gtsam/geometry/CalibratedCamera.h>

#include <CppUnitLite/TestHarness.h>

#include <iostream>

using namespace std;
using namespace gtsam;

/* ************************************************************************* */
TEST(SlamExpressions, project2) {
  typedef Expression<CalibratedCamera> CalibratedCamera_;

  Rot3_ rot3_expr('r', 0);          // unknown rotation !
  Point3_ t_expr(Point3(1, 2, 3));  // known translation
  Pose3_ pose_expr(&Pose3::Create, rot3_expr, t_expr);
  CalibratedCamera_ camera_expr(&CalibratedCamera::Create, pose_expr);
  Point3_ point3_expr('p', 12);  // unknown 3D point with index 12, for funsies
  Point2_ point2_expr = project2<CalibratedCamera>(camera_expr, point3_expr);

  // Set the linearization point
  Values values;
  values.insert(Symbol('r', 0), Rot3());
  values.insert(Symbol('p', 12), Point3(4, 5, 6));

  EXPECT_CORRECT_EXPRESSION_JACOBIANS(point2_expr, values, 1e-7, 1e-5);
}

/* ************************************************************************* */
int main() {
  TestResult tr;
  return TestRegistry::runAllTests(tr);
}
/* ************************************************************************* */
